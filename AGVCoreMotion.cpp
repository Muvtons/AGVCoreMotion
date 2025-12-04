#include "AGVCoreMotion.h"
#include "AGVCoreMotion_Config.h"

using namespace AGVCoreMotionLib;

AGVCoreMotion agvMotion; // Global instance

// ISR wrappers for static context
void IRAM_ATTR AGVCoreMotion::isrLeftWrapper() {
  agvMotion.isrLeft();
}

void IRAM_ATTR AGVCoreMotion::isrRightWrapper() {
  agvMotion.isrRight();
}

void AGVCoreMotion::begin() {
  Serial.println("\n[AGVMOTION] Initializing AGV Core Motion System...");
  
  // Initialize mutex for thread safety
  motionMutex = xSemaphoreCreateMutex();
  
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
  xTaskCreatePinnedToCore(
    [](void* param) {
      AGVCoreMotion* motion = (AGVCoreMotion*)param;
      motion->core1Task(NULL);
    },
    "AGVMotionCore1",
    8192,  // Stack size
    this,
    1,     // Priority
    &core1TaskHandle,
    1      // Core 1
  );
  
  Serial.println("[AGVMOTION] ✅ Motion System started on Core 1");
}

void AGVCoreMotion::setWheelConfig(float wheelDiameterMM, float wheelBaseMM) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) == pdPASS) {
    WHEEL_DIAMETER_MM = wheelDiameterMM;
    WHEEL_BASE_MM = wheelBaseMM;
    xSemaphoreGive(motionMutex);
  }
}

void AGVCoreMotion::setSpeedConfig(float baseSpeed, float turnSpeed, float finalSpeed) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) == pdPASS) {
    BASE_SPEED = baseSpeed;
    TURN_SPEED = turnSpeed;
    FINAL_SPEED = finalSpeed;
    xSemaphoreGive(motionMutex);
  }
}

void AGVCoreMotion::setCalibration(float linearCalibration, float turnCalibration) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) == pdPASS) {
    LINEAR_CALIBRATION = linearCalibration;
    TURN_CALIBRATION = turnCalibration;
    xSemaphoreGive(motionMutex);
  }
}

void AGVCoreMotion::setMotorTrim(float leftTrim, float rightTrim) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) == pdPASS) {
    LEFT_MOTOR_TRIM = leftTrim;
    RIGHT_MOTOR_TRIM = rightTrim;
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

void AGVCoreMotion::emergencyStop() {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) != pdPASS) return;
  
  stopRobot();
  currentState = EMERGENCY_STOP;
  currentPathState = PATH_IDLE;
  
  Serial.println("!!! EMERGENCY STOP ACTIVATED !!!");
  
  if (movementCallback) {
    movementCallback("EMERGENCY_STOP");
  }
  
  xSemaphoreGive(motionMutex);
}

void AGVCoreMotion::executePath(int sourceX, int sourceY, int destX, int destY, bool loopMode, int loopCount) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) != pdPASS) return;
  
  currentSourceX = sourceX;
  currentSourceY = sourceY;
  currentDestX = destX;
  currentDestY = destY;
  this->loopMode = loopMode;
  this->loopCount = loopCount;
  currentLoop = 0;
  currentPathState = PATH_EXECUTING;
  
  Serial.printf("[PATH] Starting path: (%d,%d) to (%d,%d)%s\n",
                sourceX, sourceY, destX, destY,
                loopMode ? String(" (loop " + String(loopCount) + " times)").c_str() : "");
  
  if (movementCallback) {
    movementCallback("PATH_STARTED");
  }
  
  xSemaphoreGive(motionMutex);
}

void AGVCoreMotion::initHardware() {
  // Initialize pins
  pinMode(SPD_L, INPUT_PULLUP);
  pinMode(SPD_R, INPUT_PULLUP);
  pinMode(DIR_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(BRK_L, OUTPUT);
  pinMode(BRK_R, OUTPUT);
  
  // Attach interrupts
  attachInterrupt(digitalPinToInterrupt(SPD_L), isrLeftWrapper, RISING);
  attachInterrupt(digitalPinToInterrupt(SPD_R), isrRightWrapper, RISING);
  
  // Setup PWM
  setupPWM();
  
  // Initial stop
  stopRobot();
  
  Serial.println("[AGVMOTION] ✅ Hardware initialized");
  Serial.println("[AGVMOTION] Supported commands:");
  Serial.println("  move forward");
  Serial.println("  turnaround");
  Serial.println("  turn_left 90");
  Serial.println("  turn_right 90");
  Serial.println("  STOP/ABORT (emergency)");
  Serial.println("  PATH:sx,sy,dx,dy:MODE[:count] (from web)");
}

void AGVCoreMotion::setupPWM() {
  ledc_timer_config_t timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_12_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = 2000,
    .clk_cfg = LEDC_AUTO_CLK
  };
  ledc_timer_config(&timer);

  ledc_channel_config_t ch = {
    .gpio_num = PWM_L, .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0, .intr_type = LEDC_INTR_DISABLE,
    .timer_sel = LEDC_TIMER_0, .duty = 0, .hpoint = 0
  };
  ledc_channel_config(&ch);

  ch.gpio_num = PWM_R;
  ch.channel = LEDC_CHANNEL_1;
  ledc_channel_config(&ch);
}

void AGVCoreMotion::core1Task(void *parameter) {
  Serial.println("[CORE1] AGV Motion task started on Core 1");
  
  while(1) {
    if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(10)) == pdPASS) {
      // Process path execution
      if (currentPathState == PATH_EXECUTING || currentPathState == PATH_LOOPING) {
        executePathSegment();
      }
      
      // Update current movement state
      switch(currentState) {
        case FORWARD_MOVING:
        case FORWARD_SLOWING:
          updateForwardMovement();
          break;
          
        case TURN_MOVING:
        case TURN_SLOWING:
          updateTurnMovement();
          break;
          
        case EMERGENCY_STOP:
          currentState = IDLE;
          break;
      }
      
      xSemaphoreGive(motionMutex);
    }
    
    delay(5);  // Small delay for responsiveness
  }
}

// Motor control methods
void AGVCoreMotion::setMotorSpeeds(float left, float right) {
  left = constrain(left, 0, 100);
  right = constrain(right, 0, 100);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, (uint32_t)(left / 100.0f * 4095));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
  ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, (uint32_t)(right / 100.0f * 4095));
  ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
}

void AGVCoreMotion::stopRobot() {
  digitalWrite(BRK_L, HIGH);
  digitalWrite(BRK_R, HIGH);
  setMotorSpeeds(0, 0);
  inTurn = false;
}

void AGVCoreMotion::setDirection(bool forward) {
  digitalWrite(BRK_L, LOW);
  digitalWrite(BRK_R, LOW);
  digitalWrite(DIR_L, forward ? HIGH : LOW);
  digitalWrite(DIR_R, forward ? HIGH : LOW);
}

void AGVCoreMotion::resetEncoders() {
  noInterrupts();
  encL = encR = 0;
  interrupts();
}

// Calculation methods
long AGVCoreMotion::calculateDistancePulses(float mm) {
  return (long)((mm / (PI * WHEEL_DIAMETER_MM)) * (24 * 30) * LINEAR_CALIBRATION);
}

long AGVCoreMotion::calculateTurnPulses(float deg) {
  float arc = (PI * WHEEL_BASE_MM) * (deg / 360.0f);
  return (long)((arc / (PI * WHEEL_DIAMETER_MM)) * (24 * 30) * TURN_CALIBRATION);
}

float AGVCoreMotion::calculateRequiredTurn(int sourceX, int sourceY, int destX, int destY) {
  // Calculate angle needed to face destination
  if (sourceX == destX && sourceY == destY) return 0.0;
  
  float dx = destX - sourceX;
  float dy = destY - sourceY;
  float angle = atan2(dy, dx) * 180.0 / PI;
  
  // Convert to 0-360 range
  if (angle < 0) angle += 360.0;
  
  return angle;
}

bool AGVCoreMotion::calculateNextMovement() {
  // This is a simplified version - in a real AGV you'd need full path planning
  // For now, we'll just move in X then Y direction
  
  if (currentSourceX == currentDestX && currentSourceY == currentDestY) {
    return false; // Reached destination
  }
  
  // Move along X axis first
  if (currentSourceX != currentDestX) {
    int direction = (currentSourceX < currentDestX) ? 1 : -1;
    float distanceMM = abs(currentDestX - currentSourceX) * 1000.0; // Assuming 1 unit = 1 meter
    
    // For now, we just move forward - real implementation would handle turns
    return true;
  }
  
  // Then move along Y axis
  if (currentSourceY != currentDestY) {
    int direction = (currentSourceY < currentDestY) ? 1 : -1;
    float distanceMM = abs(currentDestY - currentSourceY) * 1000.0;
    
    return true;
  }
  
  return false;
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
  if (millis() - movementStartTime > FORWARD_TIMEOUT) {
    Serial.println(">> Forward timeout - stopping");
    stopRobot();
    currentState = IDLE;
    return;
  }
  
  // Check if we've reached the target
  if (avgPulses >= targetPulses) {
    stopRobot();
    currentState = IDLE;
    Serial.println(">> Forward completed");
    
    if (movementCallback) {
      movementCallback("FORWARD_COMPLETED");
    }
    return;
  }
  
  // Handle slowdown phase
  if (avgPulses >= slowdownStart) {
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
  if (millis() - movementStartTime > TURN_TIMEOUT) {
    Serial.println(">> Turn timeout - stopping");
    stopRobot();
    inTurn = false;
    currentState = IDLE;
    return;
  }
  
  // Check if we've reached the target
  if (avgPulses >= targetPulses) {
    stopRobot();
    inTurn = false;
    currentState = IDLE;
    Serial.println(">> Turn completed");
    
    if (movementCallback) {
      movementCallback("TURN_COMPLETED");
    }
    return;
  }
  
  // Handle slowdown phase
  if (avgPulses >= slowdownStart) {
    float prog = (float)(avgPulses - slowdownStart) / max(1L, targetPulses - slowdownStart);
    float speed = TURN_SPEED - (TURN_SPEED - FINAL_SPEED) * (prog * prog);
    setMotorSpeeds(speed, speed);
  }
}

void AGVCoreMotion::executePathSegment() {
  if (currentState != IDLE) return; // Wait for current movement to finish
  
  if (currentPathState == PATH_EXECUTING) {
    // Simple path execution: move forward 1000mm as test
    targetPulses = calculateDistancePulses(1000.0);
    slowdownStart = (long)(targetPulses * 0.8f);
    resetEncoders();
    setDirection(true);
    setMotorSpeeds(BASE_SPEED * LEFT_MOTOR_TRIM, BASE_SPEED * RIGHT_MOTOR_TRIM);
    movementStartTime = millis();
    currentState = FORWARD_MOVING;
    
    Serial.println(">> Path segment: Moving forward 1000mm");
    
    currentPathState = PATH_IDLE; // For now, just one segment
  }
  
  // Handle loop completion
  if (loopMode && currentLoop >= loopCount) {
    currentPathState = PATH_IDLE;
    loopMode = false;
    Serial.printf(">> Path loop completed (%d iterations)\n", loopCount);
    
    if (movementCallback) {
      movementCallback("PATH_COMPLETED");
    }
  }
}

// Command processing
void AGVCoreMotion::processCommand(const char* cmd, uint8_t source, uint8_t priority) {
  if (xSemaphoreTake(motionMutex, pdMS_TO_TICKS(100)) != pdPASS) return;
  
  String command = String(cmd);
  command.trim();
  
  Serial.printf("\n[MOTION] Command received: '%s' (source=%d, priority=%d)\n", 
                command.c_str(), source, priority);
  
  // Emergency commands have highest priority
  if (priority == 1 && (command.equalsIgnoreCase("STOP") || command.equalsIgnoreCase("ABORT"))) {
    emergencyStop();
    
    if (emergencyCallback) {
      emergencyCallback();
    }
    
    xSemaphoreGive(motionMutex);
    return;
  }
  
  // Only process relevant commands
  if (isValidMovementCommand(command.c_str())) {
    parseAndExecuteCommand(command.c_str());
  } else {
    Serial.printf("[MOTION] Ignoring irrelevant command: '%s'\n", command.c_str());
  }
  
  xSemaphoreGive(motionMutex);
}

bool AGVCoreMotion::isValidMovementCommand(const char* cmd) {
  String command = String(cmd);
  command.trim();
  command.toLowerCase();
  
  // Check for movement commands
  if (command.startsWith("move forward") ||
      command.startsWith("turn_left") ||
      command.startsWith("turn_right") ||
      command.startsWith("turnaround") ||
      command.equalsIgnoreCase("stop") ||
      command.equalsIgnoreCase("abort") ||
      command.startsWith("path:")) {
    return true;
  }
  
  // Check for emergency commands
  if (command.startsWith("emergency") || command.startsWith("!!!")) {
    return true;
  }
  
  return false;
}

void AGVCoreMotion::parseAndExecuteCommand(const char* cmd) {
  String command = String(cmd);
  command.trim();
  
  // Emergency stop
  if (command.equalsIgnoreCase("STOP") || command.equalsIgnoreCase("ABORT")) {
    emergencyStop();
    return;
  }
  
  // Move forward
  if (command.equalsIgnoreCase("move forward")) {
    targetPulses = calculateDistancePulses(1000.0); // Default 1000mm
    slowdownStart = (long)(targetPulses * 0.8f);
    resetEncoders();
    setDirection(true);
    setMotorSpeeds(BASE_SPEED * LEFT_MOTOR_TRIM, BASE_SPEED * RIGHT_MOTOR_TRIM);
    movementStartTime = millis();
    currentState = FORWARD_MOVING;
    Serial.println(">> Starting forward movement (1000mm)");
    return;
  }
  
  // Turnaround (360 degrees)
  if (command.equalsIgnoreCase("turnaround")) {
    targetPulses = calculateTurnPulses(360.0);
    slowdownStart = (long)(targetPulses * 0.6f);
    resetEncoders();
    inTurn = true;
    digitalWrite(BRK_L, LOW);
    digitalWrite(BRK_R, LOW);
    digitalWrite(DIR_L, HIGH); // Right turn
    digitalWrite(DIR_R, LOW);
    setMotorSpeeds(TURN_SPEED, TURN_SPEED);
    movementStartTime = millis();
    currentState = TURN_MOVING;
    Serial.println(">> Starting 360° turn");
    return;
  }
  
  // Turn left/right
  if (command.startsWith("turn_left ") || command.startsWith("turn_right ")) {
    bool isRight = command.startsWith("turn_right");
    int spaceIndex = command.indexOf(' ');
    String angleStr = command.substring(spaceIndex + 1);
    int angle = angleStr.toInt();
    
    if (angle > 0 && angle <= 360) {
      targetPulses = calculateTurnPulses(abs(angle));
      slowdownStart = (long)(targetPulses * 0.6f);
      resetEncoders();
      inTurn = true;
      digitalWrite(BRK_L, LOW);
      digitalWrite(BRK_R, LOW);
      digitalWrite(DIR_L, isRight ? HIGH : LOW);
      digitalWrite(DIR_R, isRight ? LOW : HIGH);
      setMotorSpeeds(TURN_SPEED, TURN_SPEED);
      movementStartTime = millis();
      currentState = TURN_MOVING;
      Serial.printf(">> Starting %s turn (%d°)\n", isRight ? "right" : "left", angle);
    } else {
      Serial.println(">> Invalid angle (must be 1-360)");
    }
    return;
  }
  
  // Path command from web interface
  if (command.startsWith("PATH:")) {
    // Format: PATH:sx,sy,dx,dy:MODE[:count]
    // Example: PATH:1,1,3,2:ONCE or PATH:1,1,3,2:LOOP:5
    
    int firstColon = command.indexOf(':');
    int secondColon = command.indexOf(':', firstColon + 1);
    
    if (secondColon == -1) {
      Serial.println(">> Invalid PATH command format");
      return;
    }
    
    String coords = command.substring(firstColon + 1, secondColon);
    String modePart = command.substring(secondColon + 1);
    
    // Parse coordinates
    int comma1 = coords.indexOf(',');
    int comma2 = coords.indexOf(',', comma1 + 1);
    int comma3 = coords.indexOf(',', comma2 + 1);
    
    if (comma3 == -1) {
      Serial.println(">> Invalid coordinate format in PATH command");
      return;
    }
    
    int sourceX = coords.substring(0, comma1).toInt();
    int sourceY = coords.substring(comma1 + 1, comma2).toInt();
    int destX = coords.substring(comma2 + 1, comma3).toInt();
    int destY = coords.substring(comma3 + 1).toInt();
    
    // Parse mode
    bool loopMode = false;
    int loopCount = 1;
    
    if (modePart.startsWith("LOOP")) {
      loopMode = true;
      int thirdColon = modePart.indexOf(':');
      if (thirdColon != -1) {
        loopCount = modePart.substring(thirdColon + 1).toInt();
      }
    }
    
    executePath(sourceX, sourceY, destX, destY, loopMode, loopCount);
    return;
  }
  
  Serial.printf(">> Unknown command: '%s'\n", command.c_str());
}