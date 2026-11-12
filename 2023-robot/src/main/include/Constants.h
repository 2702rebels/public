#pragma once

#include <frc/trajectory/TrapezoidProfile.h>
#include <rev/CANSparkMax.h>
#include <units/acceleration.h>
#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/current.h>
#include <units/length.h>
#include <units/velocity.h>

#include <numbers>

#include "Directives.h"

enum Alliance {
  RED,
  BLUE,
};

enum Side { INNER, OUTER };

/**
 * The Constants header provides a convenient place for teams to hold robot-wide
 * numerical or bool constants. This should not be used for any other purpose.
 *
 * It is generally a good idea to place constants into subsystem- or
 * command-specific namespaces within this header, which can then be used where
 * they are needed.
 */

namespace RobotConstants {
// Robot size accounting for bumpers
constexpr units::inch_t kBumperSize = 31_in;
}  // namespace RobotConstants

namespace DriveConstants {
// Driving Parameters
// Note that these are not the maximum capable speeds of the robot, rather the
// allowed maximum speeds
constexpr units::meters_per_second_t kMaxSpeed = 4_mps;
constexpr units::radians_per_second_t kMaxAngularSpeed { 2 * std::numbers::pi };

constexpr double kDirectionSlewRate = std::numbers::pi / 2;  // radians per second
constexpr double kAccelerateSlewRate = 0.8;                  // percent per second (1 = 100%)
constexpr double kDecelerateSlewRate = 2.0;                  // percent per second (1 = 100%)
constexpr double kRotationalSlewRate = 1.5;                  // percent per second (1 = 100%)

// Chassis configuration
// Distances between centers of wheels on robot
constexpr units::meter_t kTrackWidth = 0.5461_m;  // right and left wheels
constexpr units::meter_t kWheelBase = 0.5461_m;   // front and back wheels

// Angular offsets of the modules relative to the chassis in radians
constexpr units::radian_t kFrontLeftChassisAngularOffset { -std::numbers::pi / 2 };
constexpr units::radian_t kFrontRightChassisAngularOffset { 0 };
constexpr units::radian_t kRearLeftChassisAngularOffset { std::numbers::pi };
constexpr units::radian_t kRearRightChassisAngularOffset { std::numbers::pi / 2 };

// SPARK MAX CAN IDs
constexpr int kFrontLeftDrivingCanId = 6;
constexpr int kRearLeftDrivingCanId = 8;
constexpr int kFrontRightDrivingCanId = 3;
constexpr int kRearRightDrivingCanId = 1;

constexpr int kFrontLeftTurningCanId = 5;
constexpr int kRearLeftTurningCanId = 7;
constexpr int kFrontRightTurningCanId = 4;
constexpr int kRearRightTurningCanId = 2;
}  // namespace DriveConstants

namespace ModuleConstants {
// Invert the turning encoder, since the output shaft rotates in the opposite
// direction of the steering motor in the MAXSwerve Module.
constexpr bool kTurningEncoderInverted = true;

// The MAXSwerve module can be configured with one of three pinion gears: 12T,
// 13T, or 14T. This changes the drive speed of the module (a pinion gear with
// more teeth will result in a robot that drives faster).
constexpr int kDrivingMotorPinionTeeth = 14;

// Calculations required for driving motor conversion factors and feed forward
constexpr double kDrivingMotorFreeSpeedRps = 5676.0 / 60;  // NEO free speed is 5676 RPM
constexpr units::meter_t kWheelDiameter = 0.0762_m;
constexpr units::meter_t kWheelCircumference = kWheelDiameter * std::numbers::pi;

// 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
// teeth on the bevel pinion
constexpr double kDrivingMotorReduction = (45.0 * 22.0) / (kDrivingMotorPinionTeeth * 15.0);

constexpr double kTurningMotorFreeSpeedRps = 11000.0 / 60;  // NEO550 free speed is 11000 RPM

// azimuth ratio from REV specification
// https://www.revrobotics.com/rev-21-3005/
constexpr double KTurningMotorReduction = 9424.0 / 203.0;

constexpr double kDrivingEncoderPositionFactor =
  kWheelCircumference.value() / kDrivingMotorReduction;  // meters

constexpr double kDrivingEncoderVelocityFactor =
  kDrivingEncoderPositionFactor / 60.0;  // meters per second

constexpr double kDrivingWheelFreeSpeedRps =
  kDrivingMotorFreeSpeedRps * kDrivingEncoderPositionFactor;

constexpr double kTurningEncoderPositionFactor = (2 * std::numbers::pi);  // radians
constexpr double kTurningEncoderVelocityFactor =
  kTurningEncoderPositionFactor / 60.0;  // radians per second

constexpr units::radians_per_second_t kTurningMotorMaxAngularVelocity =
  units::radians_per_second_t {
    kTurningEncoderPositionFactor * kTurningMotorFreeSpeedRps / KTurningMotorReduction,
  };

constexpr units::radian_t kTurningEncoderPositionPIDMinInput = 0_rad;
constexpr units::radian_t kTurningEncoderPositionPIDMaxInput =
  units::radian_t { kTurningEncoderPositionFactor };

constexpr uint32_t kDrivingVelocityMeasurementPeriodMs = 20;
constexpr uint32_t kDrivingVelocityMeasurementDepth = 4;

constexpr double kDrivingP = 0.1;
constexpr double kDrivingI = 0;
constexpr double kDrivingD = 0;
constexpr double kDrivingFF =
  (1 / kDrivingWheelFreeSpeedRps) * 1.1;  // add +10% to overshoot and pull down with P
constexpr double kDrivingMinOutput = -1;
constexpr double kDrivingMaxOutput = 1;

constexpr double kTurningP = 2.5;
constexpr double kTurningI = 0;
constexpr double kTurningD = 0;
constexpr double kTurningFF = 0;
constexpr double kTurningMinOutput = -1;
constexpr double kTurningMaxOutput = 1;

constexpr rev::CANSparkMax::IdleMode kDrivingMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;
constexpr rev::CANSparkMax::IdleMode kTurningMotorIdleMode = rev::CANSparkMax::IdleMode::kBrake;

constexpr units::ampere_t kDrivingMotorCurrentLimit = 60_A;
constexpr units::ampere_t kTurningMotorCurrentLimit = 20_A;

constexpr units::volt_t kVoltageNominal = 12_V;
constexpr units::volt_t kVoltageCompensation = 12_V;
}  // namespace ModuleConstants

namespace AutoConstants {
constexpr auto kMaxSpeed = 3_mps;
constexpr auto kMaxAcceleration = 4_mps_sq;
constexpr auto kMaxDeceleration = 6_mps_sq;
constexpr auto kMaxAngularSpeed = 480_deg_per_s;
constexpr auto kMaxAngularAcceleration = 360_deg_per_s_sq;

constexpr double kPXController = 1.5;
constexpr double kIXController = 0.0;
constexpr double kDXController = 0.0;
constexpr double kPYController = 1.5;
constexpr double kIYController = 0.0;
constexpr double kDYController = 0.0;
constexpr double kPThetaController = 9.0;
constexpr double kIThetaController = 0.0;
constexpr double kDThetaController = 0.0;
}  // namespace AutoConstants

namespace OIConstants {
constexpr int kDriverControllerPort = 0;
constexpr int kOperatorControllerPort = 1;
constexpr double kDriveDeadband = 0.2;
}  // namespace OIConstants

namespace IntakeConstants {
constexpr int kIntakeLeaderControllerCanId = 10;
constexpr int kIntakeFollowerControllerCanId = 11;

constexpr double kIntakeHoldSpeed = 0.2;  // 775's can stall at 20% duty cycle for an entire match.
constexpr double kHasGamePieceCurrent = 15;

constexpr units::millisecond_t kOuttakingTime = 400_ms;

constexpr int kGripperEncoderChannel1Id = 1;
constexpr int kGripperEncoderChannel2Id = 2;
constexpr int kGripperEncoderPulsesPerRevolution = 1024;
constexpr double kGripperZeroOffset = 45.0;

constexpr double kGripperReduction = 9 / 1 * 7 / 1;
constexpr double kGripperAnglePerTick = 360.0 / (4096.0 * 10.0);

constexpr double kGripperHomeCurrent = 14;

constexpr int kIntakeSwitchId = 8;
}  // namespace IntakeConstants

namespace ArmConstants {
constexpr int kWhichRobotJumper = 9;

// Execute cycle time.
constexpr units::second_t kDt = 20_ms;

// Motor controller CAN ID's
constexpr int kExtendingArmAxis1MotorControllerCanId = 12;
constexpr int kExtendingArmAxis2MotorControllerCanId = 13;
constexpr int kExtendingArmAxis3MotorControllerCanId = 9;

constexpr int kAxis1HomeLimitSwitchId = 0;

// Self Avoidance constants
constexpr double kPastBumperAngle = 15;
constexpr double kAxis2ZeroDeadBand = 1;

// Axis 1 (Shoulder)
constexpr double kAxis1CurrentLimit = 40;
constexpr double kAxis1MinOutput = -1.0;
constexpr double kAxis1MaxOutput = 1;
// Neo into a versa 45:1 -> sprocket reduction of 26:14 -> sprocket reduction of 60:14.
constexpr double kAxis1OverallReduction = (9.0 / 1.0) * (5.0 / 1.0) * (26.0 / 14.0) * (60.0 / 14.0);
constexpr double kAxis1MotorRotationsPerDegree = kAxis1OverallReduction / 360;
constexpr double kAxis1OverallReductionComp =
  (9.0 / 1.0) * (5.0 / 1.0) * (44.0 / 14.0) * (44.0 / 15.0);
constexpr double kAxis1MotorRotationsPerDegreeComp = kAxis1OverallReductionComp / 360;
constexpr double kAxis1ZeroAngle = 0;
constexpr int kAxis1HomeTimeoutInSeconds = 3;
constexpr double kAxis1HomeCurrentThreshold = 30;
constexpr double kaxis1ffAngleCorrection = 64;
constexpr double kAxis1Deadband = 2;
constexpr units::degree_t kAxis1MaxAngle = units::degree_t(100);

// Axis 2 (Extension)
constexpr int kAxis2CurrentLimit = 25;
constexpr double kAxis2MinOutput = -1;
constexpr double kAxis2MaxOutput = 1;
// Neo 550 into versa 35:1.
constexpr double kAxis2OverallReduction = (7.0 / 1.0) * (5.0 / 1.0);
// Belt is running on an 18t 5mm HTD pulley.
constexpr double kAxis2MotorRotationsPerInchOfTravel =
  kAxis2OverallReduction / ((18.0 * 5.0) / 25.4);
constexpr double kAxis2MotorRotationsPerMeterOfTravel =
  kAxis2OverallReduction / ((18.0 * 5.0) / 1000);
constexpr int kAxis2HomeTimeoutInSeconds = 3;
constexpr double kAxis2HomeCurrentThreshold = 15;
constexpr double kAxis2Deadband = 1.2;
constexpr units::inch_t kAxis2MaxExtension = 17.75_in;

// Position
constexpr units::degree_t kConePlacementDistance = 12.0_deg;

}  // namespace ArmConstants

namespace LEDConstants {
// Execute cycle time.
constexpr int kpwmPort = 0;
constexpr int kLength = 66;
}  // namespace LEDConstants
