#ifndef AGVCOREMOTION_CONFIG_H
#define AGVCOREMOTION_CONFIG_H

// Default configuration values - used in AGVCoreMotion::begin()
#define DEFAULT_WHEEL_DIAMETER_MM   200.0f    // mm
#define DEFAULT_WHEEL_BASE_MM       600.0f    // distance between wheels (mm)
#define DEFAULT_BASE_SPEED          65.0f     // forward movement speed (%)
#define DEFAULT_TURN_SPEED          50.0f     // turning speed (%)
#define DEFAULT_FINAL_SPEED         5.0f      // slowdown speed at end (%)
#define DEFAULT_LINEAR_CALIBRATION  0.895f    // distance calibration factor
#define DEFAULT_TURN_CALIBRATION    0.55f     // turn calibration factor
#define DEFAULT_LEFT_MOTOR_TRIM     1.00f     // left motor speed adjustment
#define DEFAULT_RIGHT_MOTOR_TRIM    1.00f     // right motor speed adjustment

// Safety limits - prevent dangerous configurations
#define MIN_WHEEL_DIAMETER_MM       50.0f
#define MAX_WHEEL_DIAMETER_MM       500.0f
#define MIN_WHEEL_BASE_MM           200.0f
#define MAX_WHEEL_BASE_MM           1000.0f
#define MIN_MOTOR_SPEED             20.0f
#define MAX_MOTOR_SPEED             100.0f
#define MIN_CALIBRATION_FACTOR      0.5f
#define MAX_CALIBRATION_FACTOR      1.5f
#define MIN_MOTOR_TRIM              0.8f
#define MAX_MOTOR_TRIM              1.2f

// Movement timeouts (milliseconds)
#define MOVEMENT_TIMEOUT_MS         15000     // 15 seconds max for any movement

#endif
