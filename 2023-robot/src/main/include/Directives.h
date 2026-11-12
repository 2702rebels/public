// Indicates that auto commands should log telemetry data
#define LOG_AUTO_COMMAND

// Indicates that vision targets distances and angles must be logged
#define LOG_VISION_TARGETS

// Indicates that detailed logging of DS and joystick controls should be enabled
#define LOG_DATA_DRIVER_STATION

// Indicates that verbose log statements should be written.
// (Mostly for debugging. This will likely spam logs.)
// #define VERBOSE_LOG

// Indicates if trace log statements should be written.
// #define TRACE_LOG

// Indicates if the drivetrain commands are bound.
// Currently the drivetrain object is still instantiated.
#define DRIVETRAIN_ENABLED

// Indicates use of second-order swerve kinematics
#define SWERVE_SECOND_ORDER_KINEMATICS

// Indicates that drivetrain odometry should use pose estimation coupled with
// an external vision system that can provide global positioning estimates;
// if not enabled, then basic odometry tracking based on encoders and gyro only
// will be used to estimate robot position
// #define DRIVETRAIN_ODOMETRY_VISION

// Indicates if the extending arm is instantiated and it's commands are bound.
#define ARM_ENABLED

// Indicates if the intake is instantiated and it's commands are bound.
#define INTAKE_ENABLED

// Indicates that autonomous mode chooser via Shuffleboard should be used
#define AUTO_CHOOSER

// Indicates that the led's are enabled.
#define LED_ENABLED

#define OPERATOR_CONTROLLER_ENABLED

#define INTAKE_SWITCH_ENABLED

#define ARMFEEDFORWARDS_ENABLED
