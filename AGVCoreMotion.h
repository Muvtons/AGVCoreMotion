#ifndef AGVCOREMOTION_H
#define AGVCOREMOTION_H

#include <Arduino.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/semphr.h>

namespace AGVCoreMotionLib {

// Movement states for state machine
enum MovementState {
  IDLE,
  FORWARD_MOVING,
  FORWARD_SLOWING,
  TURN_MOVING,
  TURN_SLOWING,
  EMERGENCY_STOP
};

// Simplified - removed path states since we're using direct commands only
class AGVCoreMotion {
public:
  // Callback function types
  typedef void (*MovementCallback)(const char* status);
  typedef void (*EmergencyCallback)();
  
  // Initialize the motion system - FIXED SIGNATURE
  void begin(uint8_t spdLeftPin = 4, uint8_t spdRightPin = 5,
             uint8_t pwmLeftPin = 6, uint8_t pwmRightPin = 7,
             uint8_t dirLeftPin = 15, uint8_t dirRightPin = 16,
             uint8_t brkLeftPin = 17, uint8_t brkRightPin = 18);
  
  // Configure robot parameters - optional but recommended
  void setWheelConfig(float wheelDiameterMM = 200.0, float wheelBaseMM = 600.0);
  void setSpeedConfig(float baseSpeed = 65.0, float turnSpeed = 50.0, float finalSpeed = 5.0);
  void setCalibration(float linearCalibration = 0.895, float turnCalibration = 0.55);
  void setMotorTrim(float leftTrim = 1.00, float rightTrim = 1.00);
  
  // Set callbacks for status updates and emergencies
  void setMovementCallback(MovementCallback callback);
  void setEmergencyCallback(EmergencyCallback callback);
  
  // Emergency stop (highest priority) and recovery
  void emergencyStop();
  void clearEmergency();
  bool isInEmergency() const { return currentState == EMERGENCY_STOP; }
  
  // Process command from serial interface - SIMPLIFIED
  void processCommand(const char* cmd);
  
  // Direct movement commands (for internal use)
  void moveForward(float distanceMM = 1000.0);
  void turnLeft(float degrees = 90.0);
  void turnRight(float degrees = 90.0);
  void turnAround();

private:
  // Hardware pins - now configurable
  uint8_t SPD_L, SPD_R, PWM_L, PWM_R, DIR_L, DIR_R, BRK_L, BRK_R;
  
  // Robot configuration
  float WHEEL_DIAMETER_MM;
  float WHEEL_BASE_MM;
  float BASE_SPEED;
  float TURN_SPEED;
  float FINAL_SPEED;
  float LINEAR_CALIBRATION;
  float TURN_CALIBRATION;
  float LEFT_MOTOR_TRIM;
  float RIGHT_MOTOR_TRIM;
  
  // Movement state
  volatile long encL = 0;
  volatile long encR = 0;
  MovementState currentState = IDLE;
  
  // Timing and targets
  long targetPulses = 0;
  long slowdownStart = 0;
  unsigned long movementStartTime = 0;
  static constexpr unsigned long MOVEMENT_TIMEOUT = 15000; // 15 seconds max
  
  // Callbacks
  MovementCallback movementCallback = nullptr;
  EmergencyCallback emergencyCallback = nullptr;
  
  // Task handles
  TaskHandle_t core1TaskHandle = nullptr;
  SemaphoreHandle_t motionMutex = nullptr;
  
  // Internal methods
  void core1Task(void *parameter);
  void initHardware();
  void setupPWM();
  
  // ISR methods
  static void IRAM_ATTR isrLeftWrapper();
  static void IRAM_ATTR isrRightWrapper();
  void IRAM_ATTR isrLeft() { encL++; }
  void IRAM_ATTR isrRight() { encR++; }
  
  // Motor control
  void setMotorSpeeds(float left, float right);
  void stopRobot();
  void setDirection(bool forward);
  void resetEncoders();
  
  // Calculation methods
  long calculateDistancePulses(float mm);
  long calculateTurnPulses(float deg);
  
  // State machine methods
  void updateForwardMovement();
  void updateTurnMovement();
  
  // Command validation
  bool isValidCommand(const char* cmd) const;
};

} // namespace AGVCoreMotionLib

// Global instance for easy access
extern AGVCoreMotionLib::AGVCoreMotion agvMotion;

#endif
