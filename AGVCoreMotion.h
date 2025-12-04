#ifndef AGVCOREMOTION_H
#define AGVCOREMOTION_H

#include <Arduino.h>
#include <driver/ledc.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>
#include <freertos/semphr.h>

// Unique library namespace to prevent conflicts
namespace AGVCoreMotionLib {

// Movement states for state machine
enum MovementState {
  IDLE,
  FORWARD_START,
  FORWARD_MOVING,
  FORWARD_SLOWING,
  TURN_START,
  TURN_MOVING,
  TURN_SLOWING,
  EMERGENCY_STOP
};

// Path execution states
enum PathState {
  PATH_IDLE,
  PATH_EXECUTING,
  PATH_LOOPING
};

class AGVCoreMotion {
public:
  // Callback function types
  typedef void (*MovementCallback)(const char* status);
  typedef void (*EmergencyCallback)();
  
  // Initialize the motion system - 1 of 4 lines
  void begin();
  
  // Configure robot parameters - optional but recommended
  void setWheelConfig(float wheelDiameterMM = 200.0, float wheelBaseMM = 600.0);
  void setSpeedConfig(float baseSpeed = 65.0, float turnSpeed = 50.0, float finalSpeed = 5.0);
  void setCalibration(float linearCalibration = 0.895, float turnCalibration = 0.55);
  void setMotorTrim(float leftTrim = 1.00, float rightTrim = 1.00);
  
  // Set callbacks for status updates and emergencies
  void setMovementCallback(MovementCallback callback);
  void setEmergencyCallback(EmergencyCallback callback);
  
  // Emergency stop (highest priority)
  void emergencyStop();
  
  // Path execution (from web interface)
  void executePath(int sourceX, int sourceY, int destX, int destY, bool loopMode = false, int loopCount = 1);
  
  // Library handles everything else automatically on Core 1
  void loop(); // Called automatically in background task

private:
  // Hardware configuration
  float WHEEL_DIAMETER_MM;
  float WHEEL_BASE_MM;
  float BASE_SPEED;
  float TURN_SPEED;
  float FINAL_SPEED;
  float LINEAR_CALIBRATION;
  float TURN_CALIBRATION;
  float LEFT_MOTOR_TRIM;
  float RIGHT_MOTOR_TRIM;
  
  // Hardware pins
  const uint8_t SPD_L = 4;
  const uint8_t SPD_R = 5;
  const uint8_t PWM_L = 6;
  const uint8_t PWM_R = 7;
  const uint8_t DIR_L = 15;
  const uint8_t DIR_R = 16;
  const uint8_t BRK_L = 17;
  const uint8_t BRK_R = 18;
  
  // Movement state
  volatile long encL = 0;
  volatile long encR = 0;
  volatile bool inTurn = false;
  MovementState currentState = IDLE;
  PathState currentPathState = PATH_IDLE;
  
  // Path execution variables
  int currentSourceX = 1;
  int currentSourceY = 1;
  int currentDestX = 1;
  int currentDestY = 1;
  bool loopMode = false;
  int loopCount = 1;
  int currentLoop = 0;
  
  // Timing and targets
  long targetPulses = 0;
  long slowdownStart = 0;
  unsigned long movementStartTime = 0;
  const unsigned long FORWARD_TIMEOUT = 10000;
  const unsigned long TURN_TIMEOUT = 15000;
  
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
  void isrLeft() { encL++; }
  void isrRight() { encR++; }
  
  // Motor control
  void setMotorSpeeds(float left, float right);
  void stopRobot();
  void setDirection(bool forward);
  void resetEncoders();
  
  // Calculation methods
  long calculateDistancePulses(float mm);
  long calculateTurnPulses(float deg);
  float calculateRequiredTurn(int sourceX, int sourceY, int destX, int destY);
  bool calculateNextMovement();
  
  // State machine methods
  void updateForwardMovement();
  void updateTurnMovement();
  void executePathSegment();
  
  // Command processing
  void processCommand(const char* cmd, uint8_t source, uint8_t priority);
  bool isValidMovementCommand(const char* cmd);
  void parseAndExecuteCommand(const char* cmd);
};

} // namespace AGVCoreMotionLib

// Global instance for easy access
extern AGVCoreMotionLib::AGVCoreMotion agvMotion;

#endif
