#include "AGVCoreMotion.h"
#include <Arduino.h>

using namespace AGVCoreMotionLib;

AGVCoreMotion agvMotion; // Global instance

// ISR wrappers for static context
void IRAM_ATTR AGVCoreMotion::isrLeftWrapper() {
  agvMotion.isrLeft();
}

void IRAM_ATTR AGVCoreMotion::isrRightWrapper() {
  agvMotion.isrRight();
}

void AGVCoreMotion::begin(uint8_t spdLeftPin, uint8_t spdRightPin,
                         uint8_t pwmLeftPin, uint8_t pwmRightPin,
                         uint8_t dirLeftPin, uint8_t dirRightPin,
                         uint8_t brkLeftPin, uint8_t brkRightPin) {
  Serial.println("\n[AGVMOTION] Initializing AGV Core Motion System...");
  
  // Store pin configuration
  SPD_L = spdLeftPin;
  SPD_R = spdRightPin;
  PWM_L = pwmLeftPin;
  PWM_R = pwmRightPin;
  DIR_L = dirLeftPin;
  DIR_R = dirRightPin;
  BRK_L = brkLeftPin;
  BRK_R = brkRightPin;
  
  // Initialize mutex for thread safety
  motionMutex = xSemaphoreCreateMutex();
  if (!motionMutex) {
    Serial.println("[ERROR] Failed to create motion mutex!");
    return;
  }
  
  // Set default configuration
  WHEEL_DIAMETER_MM = 200.0;
  WHEEL_BASE_MM = 600.0;
  BASE_SPEED = 65.0;
  TURN_SPEED = 50.0;
  FINAL_SPEED = 5.0;
  LINEAR_CALIBRATION = 0.895;
  TURN_CALIBRATION = 0.55;
  LEFT_MOTOR_TRIM = 1.00;
  RIGHT_MOTOR_TRIM = 1.00;
  
  // Initialize hardware
  initHardware();
  
  // Start Core 1 task (handles all motor control)
  if (xTaskCreatePinnedToCore(
    [](void* param) {
      AGVCoreMotion* motion = (AGVCoreMotion*)param;
      motion->core1Task(NULL);
    },
    "AGVMotionCore1",
    8192,  // Stack size
    this,
    configMAX_PRIORITIES - 1,     // High priority
    &core1TaskHandle,
    1      // Core 1
  ) != pdPASS) {
    Serial.println("[ERROR] Failed to create Core 1 task!");
  }
  
  Serial.println("[AGVMOTION] ✅ Motion System started on Core 1");
  Serial.println("[AGVMOTION] ✅ Ready for serial commands");
  Serial.println("[AGVMOTION] Supported commands:");
  Serial.println("  move forward");
  Serial.println("  turn_left [degrees]");
  Serial.println("  turn_right [degrees]");
  Serial.println("  turnaround");
  Serial.println("  STOP (emergency)");
  Serial.println("  CLEAR_EMERGENCY");
}

void AGVCoreMotion::initHardware() {
  // Initialize pins with error checking
  pinMode(SPD_L, INPUT_PULLUP);
  pinMode(SPD_R, INPUT_PULLUP);
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(BRK_L, OUTPUT);
  pinMode(BRK_R, OUTPUT);
  
  // Initial safe state
  digitalWrite(BRK_L, HIGH);
  digitalWrite(BRK_R, HIGH);
  digitalWrite(DIR_L, LOW);
  digitalWrite(DIR_R, LOW);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(SPD_L), isrLeftWrapper, RISING);
  attachInterrupt(digitalPinToInterrupt(SPD_R), isrRightWrapper, RISING);
  
  // Setup PWM
  setupPWM();
  
  Serial.println("[AGVMOTION] ✅ Hardware initialized");
}

void AGVCoreMotion::setupPWM() {
  ledc_timer_config_t timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 20000,  // 20kHz for quieter operation
    .clk_cfg = LEDC_AUTO_CLK
  };
  
  if (ledc_timer_config(&timer) != ESP_OK) {
    Serial.println("[ERROR] PWM timer configuration failed!");
    return;
  }

  ledc_channel_config_t ch = {
    .gpio_num = PWM_L, 
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0, 
    .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0, 
    .duty = 0, 
    .hpoint = 0
  };
  
  if (ledc_channel_config(&ch) != ESP_OK) {
    Serial.println("[ERROR] Left PWM channel configuration failed!");
  }

  ch.gpio_num = PWM_R;
  ch.channel = LEDC_CHANNEL_1;
  if (ledc_channel_config(&ch) != ESP_OK) {
    Serial.println("[ERROR] Right PWM channel configuration failed!");
  }
  
  Serial.println("[AGVMOTION] ✅ PWM configured at 20kHz");
}

void AGVCoreMotion::core1Task(void *parameter) {
  Serial.println("[CORE1] AGV Motion task started on Core 1");
  
  while(1) {
    if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(20)) == pdPASS) {
      // Only process movement if not in emergency state
      if (currentState != EMERGENCY_STOP) {
        switch(currentState) {
          case FORWARD_MOVING:
          case FORWARD_SLOWING:
            updateForwardMovement();
            break;
            
          case TURN_MOVING:
          case TURN_SLOWING:
            updateTurnMovement();
            break;
            
          default:
            break;
        }
      }
      xSemaphoreGive(motionMutex);
    }
    
    delay(5);  // Small delay for responsiveness
  }
}

// Motor control methods
void AGVCoreMotion::setMotorSpeeds(float left, float right) {
  // Apply safety limits
  left = constrain(left, 0, 100);
  right = constrain(right, 0, 100);
  
  // Convert to 12-bit duty cycle (0-4095)
  uint32_t leftDuty = (uint32_t)(left / 100.0f * 4095.0f);
  uint32_t rightDuty = (uint32_t)(right / 100.0f * 4095.0f);
  
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, leftDuty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, rightDuty);
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void AGVCoreMotion::stopRobot() {
  digitalWrite(BRK_L, HIGH);
  digitalWrite(BRK_R, HIGH);
  setMotorSpeeds(0, 0);
}

void AGVCoreMotion::setDirection(bool forward) {
  digitalWrite(BRK_L, LOW);
  digitalWrite(BRK_R, LOW);
  digitalWrite(DIR_L, forward ? HIGH : LOW);
  digitalWrite(DIR_R, forward ? HIGH : LOW);
}

void AGVCoreMotion::resetEncoders() {
  noInterrupts();
  encL = 0;
  encR = 0;
  interrupts();
}

// Calculation methods
long AGVCoreMotion::calculateDistancePulses(float mm) {
  // Pulses per revolution = 24 (encoder) * 30 (gear ratio)
  // Distance per revolution = PI * wheel diameter
  // Calibration factor accounts for slippage and mechanical tolerances
  return (long)((mm / (PI * WHEEL_DIAMETER_MM)) * (24 * 30) * LINEAR_CALIBRATION);
}

long AGVCoreMotion::calculateTurnPulses(float deg) {
  // Arc length = (PI * wheelBase) * (degrees / 360)
  return (long)((deg / 360.0f) * (PI * WHEEL_BASE_MM) / (PI * WHEEL_DIAMETER_MM) * (24 * 30) * TURN_CALIBRATION);
}

// State machine methods
void AGVCoreMotion::updateForwardMovement() {
  long l, r;
  noInterrupts();
  l = encL;
  r = encR;
  interrupts();
  
  long avgPulses = (l + r) / 2;
  
  // Check for timeout
  if (millis() - movementStartTime > MOVEMENT_TIMEOUT) {
    Serial.println(">> Forward timeout - emergency stopping");
    emergencyStop();
    return;
  }
  
  // Check if we've reached the target
  if (avgPulses >= targetPulses) {
    stopRobot();
    currentState = IDLE;
    Serial.println(">> Forward completed");
    
    if (movementCallback) {
      // Release mutex before callback to prevent deadlock
      xSemaphoreGive(motionMutex);
      movementCallback("FORWARD_COMPLETED");
      xSemaphoreTake(motionMutex, portMAX_DELAY);
    }
    return;
  }
  
  // Handle slowdown phase
  if (avgPulses >= slowdownStart && currentState != FORWARD_SLOWING) {
    currentState = FORWARD_SLOWING;
  }
  
  if (currentState == FORWARD_SLOWING) {
    float prog = (float)(avgPulses - slowdownStart) / max(1L, targetPulses - slowdownStart);
    float speed = BASE_SPEED - (BASE_SPEED - FINAL_SPEED) * constrain(prog, 0, 1);
    setMotorSpeeds(speed * LEFT_MOTOR_TRIM, speed * RIGHT_MOTOR_TRIM);
  }
}

void AGVCoreMotion::updateTurnMovement() {
  long l, r;
  noInterrupts();
  l = abs(encL);
  r = abs(encR);
  interrupts();
  
  long avgPulses = (l + r) / 2;
  
  // Check for timeout
  if (millis() - movementStartTime > MOVEMENT_TIMEOUT) {
    Serial.println(">> Turn timeout - emergency stopping");
    emergencyStop();
    return;
  }
  
  // Check if we've reached the target
  if (avgPulses >= targetPulses) {
    stopRobot();
    currentState = IDLE;
    Serial.println(">> Turn completed");
    
    if (movementCallback) {
      // Release mutex before callback to prevent deadlock
      xSemaphoreGive(motionMutex);
      movementCallback("TURN_COMPLETED");
      xSemaphoreTake(motionMutex, portMAX_DELAY);
    }
    return;
  }
  
  // Handle slowdown phase
  if (avgPulses >= slowdownStart && currentState != TURN_SLOWING) {
    currentState = TURN_SLOWING;
  }
  
  if (currentState == TURN_SLOWING) {
    float prog = (float)(avgPulses - slowdownStart) / max(1L, targetPulses - slowdownStart);
    float speed = TURN_SPEED - (TURN_SPEED - FINAL_SPEED) * (prog * prog); // Quadratic slowdown
    setMotorSpeeds(speed, speed);
  }
}

// Direct movement commands
void AGVCoreMotion::moveForward(float distanceMM) {
  if (isInEmergency()) {
    Serial.println(">> Cannot move: Emergency state active");
    return;
  }
  
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println(">> Failed to acquire motion mutex");
    return;
  }
  
  // Calculate movement parameters
  targetPulses = calculateDistancePulses(distanceMM);
  slowdownStart = (long)(targetPulses * 0.8f); // Start slowing at 80%
  resetEncoders();
  setDirection(true); // Forward
  
  // Apply motor trims and set initial speed
  float leftSpeed = BASE_SPEED * LEFT_MOTOR_TRIM;
  float rightSpeed = BASE_SPEED * RIGHT_MOTOR_TRIM;
  setMotorSpeeds(leftSpeed, rightSpeed);
  
  movementStartTime = millis();
  currentState = FORWARD_MOVING;
  
  Serial.printf(">> Starting forward movement (%.1fmm)\n", distanceMM);
  
  xSemaphoreGive(motionMutex);
}

void AGVCoreMotion::turnLeft(float degrees) {
  if (isInEmergency()) {
    Serial.println(">> Cannot turn: Emergency state active");
    return;
  }
  
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println(">> Failed to acquire motion mutex");
    return;
  }
  
  // Calculate movement parameters
  targetPulses = calculateTurnPulses(degrees);
  slowdownStart = (long)(targetPulses * 0.7f); // Start slowing at 70%
  resetEncoders();
  
  // Set direction for left turn (left motor backward, right motor forward)
  digitalWrite(BRK_L, LOW);
  digitalWrite(BRK_R, LOW);
  digitalWrite(DIR_L, LOW);  // Left motor backward
  digitalWrite(DIR_R, HIGH); // Right motor forward
  
  // Set turn speed
  setMotorSpeeds(TURN_SPEED, TURN_SPEED);
  
  movementStartTime = millis();
  currentState = TURN_MOVING;
  
  Serial.printf(">> Starting left turn (%.1f°)\n", degrees);
  
  xSemaphoreGive(motionMutex);
}

void AGVCoreMotion::turnRight(float degrees) {
  if (isInEmergency()) {
    Serial.println(">> Cannot turn: Emergency state active");
    return;
  }
  
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println(">> Failed to acquire motion mutex");
    return;
  }
  
  // Calculate movement parameters
  targetPulses = calculateTurnPulses(degrees);
  slowdownStart = (long)(targetPulses * 0.7f); // Start slowing at 70%
  resetEncoders();
  
  // Set direction for right turn (left motor forward, right motor backward)
  digitalWrite(BRK_L, LOW);
  digitalWrite(BRK_R, LOW);
  digitalWrite(DIR_L, HIGH); // Left motor forward
  digitalWrite(DIR_R, LOW);  // Right motor backward
  
  // Set turn speed
  setMotorSpeeds(TURN_SPEED, TURN_SPEED);
  
  movementStartTime = millis();
  currentState = TURN_MOVING;
  
  Serial.printf(">> Starting right turn (%.1f°)\n", degrees);
  
  xSemaphoreGive(motionMutex);
}

void AGVCoreMotion::turnAround() {
  turnRight(360.0);
}

// Emergency handling
void AGVCoreMotion::emergencyStop() {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) != pdPASS) return;
  
  stopRobot();
  currentState = EMERGENCY_STOP;
  
  Serial.println("!!! EMERGENCY STOP ACTIVATED !!!");
  
  // Release mutex BEFORE callbacks to prevent deadlock
  xSemaphoreGive(motionMutex);
  
  if (emergencyCallback) {
    emergencyCallback();
  }
  
  if (movementCallback) {
    movementCallback("EMERGENCY_STOP");
  }
}

void AGVCoreMotion::clearEmergency() {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) != pdPASS) return;
  
  if (currentState == EMERGENCY_STOP) {
    currentState = IDLE;
    Serial.println("[AGVMOTION] Emergency state cleared");
  }
  
  // Release mutex BEFORE callbacks
  xSemaphoreGive(motionMutex);
  
  if (movementCallback) {
    movementCallback("SYSTEM_NORMAL");
  }
}

// Command processing
bool AGVCoreMotion::isValidCommand(const char* cmd) const {
  if (!cmd || strlen(cmd) == 0) return false;
  
  String command = String(cmd);
  command.trim();
  command.toLowerCase();
  
  return (command.startsWith("move forward") ||
          command.startsWith("turn_left") ||
          command.startsWith("turn_right") ||
          command.equalsIgnoreCase("turnaround") ||
          command.equalsIgnoreCase("stop") ||
          command.equalsIgnoreCase("clear_emergency"));
}

void AGVCoreMotion::processCommand(const char* cmd) {
  if (!cmd || strlen(cmd) == 0) return;
  
  String command = String(cmd);
  command.trim();
  
  Serial.printf("\n[MOTION] Processing command: '%s'\n", command.c_str());
  
  // Emergency commands have highest priority
  if (command.equalsIgnoreCase("STOP") || command.equalsIgnoreCase("ABORT")) {
    emergencyStop();
    return;
  }
  
  // Clear emergency state
  if (command.equalsIgnoreCase("CLEAR_EMERGENCY")) {
    clearEmergency();
    return;
  }
  
  // Only process valid commands
  if (!isValidCommand(command.c_str())) {
    Serial.printf("[MOTION] Invalid command: '%s'\n", command.c_str());
    return;
  }
  
  // Only process movement commands if not in emergency state
  if (isInEmergency()) {
    Serial.println("[MOTION] Command blocked: Emergency state active");
    return;
  }
  
  // Process command with mutex protection
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) != pdPASS) {
    Serial.println("[MOTION] Failed to acquire mutex for command");
    return;
  }
  
  // Move forward (default 1000mm)
  if (command.equalsIgnoreCase("move forward")) {
    moveForward(1000.0);
    xSemaphoreGive(motionMutex);
    return;
  }
  
  // Turnaround (360 degrees)
  if (command.equalsIgnoreCase("turnaround")) {
    turnAround();
    xSemaphoreGive(motionMutex);
    return;
  }
  
  // Turn left/right with angle parsing
  if (command.startsWith("turn_left ") || command.startsWith("turn_right ")) {
    bool isRight = command.startsWith("turn_right");
    int spaceIndex = command.indexOf(' ');
    String angleStr = command.substring(spaceIndex + 1);
    float angle = angleStr.toFloat();
    
    if (angle > 0 && angle <= 360) {
      xSemaphoreGive(motionMutex); // Release before movement
      if (isRight) {
        turnRight(angle);
      } else {
        turnLeft(angle);
      }
      return;
    } else {
      xSemaphoreGive(motionMutex);
      Serial.println(">> Invalid angle (must be 1-360)");
      return;
    }
  }
  
  xSemaphoreGive(motionMutex);
  Serial.printf("[MOTION] Unhandled command: '%s'\n", command.c_str());
}

// Configuration methods
void AGVCoreMotion::setWheelConfig(float wheelDiameterMM, float wheelBaseMM) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) == pdPASS) {
    WHEEL_DIAMETER_MM = wheelDiameterMM;
    WHEEL_BASE_MM = wheelBaseMM;
    xSemaphoreGive(motionMutex);
  }
}

void AGVCoreMotion::setSpeedConfig(float baseSpeed, float turnSpeed, float finalSpeed) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) == pdPASS) {
    BASE_SPEED = constrain(baseSpeed, 20.0, 100.0);
    TURN_SPEED = constrain(turnSpeed, 20.0, 100.0);
    FINAL_SPEED = constrain(finalSpeed, 1.0, 20.0);
    xSemaphoreGive(motionMutex);
  }
}

void AGVCoreMotion::setCalibration(float linearCalibration, float turnCalibration) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) == pdPASS) {
    LINEAR_CALIBRATION = constrain(linearCalibration, 0.5, 1.5);
    TURN_CALIBRATION = constrain(turnCalibration, 0.5, 1.5);
    xSemaphoreGive(motionMutex);
  }
}

void AGVCoreMotion::setMotorTrim(float leftTrim, float rightTrim) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) == pdPASS) {
    LEFT_MOTOR_TRIM = constrain(leftTrim, 0.8, 1.2);
    RIGHT_MOTOR_TRIM = constrain(rightTrim, 0.8, 1.2);
    xSemaphoreGive(motionMutex);
  }
}

void AGVCoreMotion::setMovementCallback(MovementCallback callback) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) == pdPASS) {
    movementCallback = callback;
    xSemaphoreGive(motionMutex);
    Serial.println("[AGVMOTION] Movement callback registered");
  }
}

void AGVCoreMotion::setEmergencyCallback(EmergencyCallback callback) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) == pdPASS) {
    emergencyCallback = callback;
    xSemaphoreGive(motionMutex);
    Serial.println("[AGVMOTION] Emergency callback registered");
  }
}
