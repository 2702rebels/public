#pragma once

#include <ctre/Phoenix.h>
#include <frc/DriverStation.h>
#include <frc/controller/PIDController.h>
#include <frc/estimator/SwerveDrivePoseEstimator.h>
#include <frc/filter/SlewRateLimiter.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/kinematics/SwerveDriveOdometry.h>
#include <frc/smartdashboard/Field2d.h>
#include <frc2/command/SubsystemBase.h>
#include <networktables/DoubleArrayTopic.h>
#include <networktables/NetworkTable.h>
#include <networktables/NetworkTableInstance.h>

#include <optional>

#include "Constants.h"
#include "MAXSwerveModule.h"
#include "utils/AngleSlewRateLimiter.h"
#include "utils/SwerveDriveKinematics2.h"

#ifdef SWERVE_SECOND_ORDER_KINEMATICS
template <size_t NumModules>
using SwerveDriveKinematics = SwerveDriveKinematics2<NumModules>;
#else
template <size_t NumModules>
using SwerveDriveKinematics = frc::SwerveDriveKinematics<NumModules>;
#endif

// number of MAXSwerveModule managed by drive subsystem
#define NUMBER_OF_MODULES 4

class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  /**
   * Custom subsystem telemetry.
   */
  void InitSendable(wpi::SendableBuilder& builder) override;

  /**
   * Will be called periodically whenever the CommandScheduler runs.
   */
  void Periodic() override;

  /**
   * Drives the robot at given x, y and theta speeds. Speeds range from [-1, 1]
   * and the linear speeds have no effect on the angular speed.
   *
   * @param xSpeed Input speed of the robot in the x direction (forward/backwards).
   * @param ySpeed Input speed of the robot in the y direction (sideways).
   * @param rotSpeed Angular speed of the robot.
   * @param fieldRelative Indicates that provided x and y speeds are relative to the field.
   */
  void Drive(double xSpeed, double ySpeed, double rotSpeed, bool fieldRelative);

  /**
   * Re-reads and re-assigns various tuning parameters.
   */
  void UpdateTuningParameters();

  /**
   * Stops and sets the wheels into an X formation to prevent movement.
   */
  void StopX();

  /**
   * Resets the drive encoders to currently read a position of 0.
   */
  void ResetEncoders();

  /**
   * Returns the heading of the robot based on the estimated pose.
   * @return The robot's heading.
   */
  frc::Rotation2d GetHeading() const {
    return GetPose().Rotation();
  }

  /**
   * Zeroes the heading of the robot.
   */
  void ZeroHeading();

  /**
   * Returns the turn rate of the robot.
   * @return The turn rate of the robot.
   */
  units::radians_per_second_t GetTurnRate() const {
    return units::degrees_per_second_t { -m_gyroSign * m_gyro.GetRate() };
  }

  /**
   * Returns the yaw heading of the robot based on the gyro.
   */
  frc::Rotation2d GetYaw() const {
    return frc::Rotation2d(GetGyroAngle());
  }

  /**
   * Returns robot pitch.
   *
   * - negative - leans forwards
   * - positive - leans backwards
   */
  units::radian_t GetPitch() const {
    return m_pitch;
  }

  /**
   * Returns robot roll.
   *
   * - negative - banks left
   * - positive - banks right
   */
  units::radian_t GetRoll() const {
    return m_roll;
  }

  /**
   * Returns robot pitch velocity.
   */
  units::radians_per_second_t GetPitchVelocity() const {
    return m_pitchVelocity;
  }

  /**
   * Returns robot roll velocity.
   */
  units::radians_per_second_t GetRollVelocity() const {
    return m_rollVelocity;
  }

  /**
   * Returns fused pitch/roll accounting for pose.
   *
   * - negative - leans forwards
   * - positive - leans backwards
   */
  units::radian_t GetInclination() const {
    return m_inclination;
  }

  /**
   * Returns instantaneous change in inclination per time.
   */
  units::radians_per_second_t GetInclinationVelocity() const {
    return m_inclinationVelocity;
  }

  /**
   * Returns the currently-estimated pose of the robot.
   * @return The pose.
   */
  frc::Pose2d GetPose() const;

  /**
   * Returns the pose received from the vision.
   * @return The pose in "alliance wall" coordinates.
   */
  const frc::Pose2d& GetVisionPose() const {
    return m_visionPose;
  }

  /**
   * Gets the drive deadband.
   */
  double GetDeadband() const {
    return m_deadband;
  }

  /**
   * Resets the odometry to the specified pose.
   *
   * @param pose The pose to which to set the odometry.
   */
  void ResetOdometry(const frc::Pose2d& pose);

  /**
   * Stops the robot.
   */
  void Stop();

  /**
   * Enables orientation lock to the specified angle.
   */
  void EnableOrientationLock(units::radian_t angle);

  /**
   * Toggles orientation lock.
   */
  void ToggleOrientationLock(units::radian_t angle);

  /**
   * Disables orientation lock.
   */
  void DisableOrientationLock();

  /**
   * Determines whether orientation lock is engaged.
   */
  bool HasOrientationLock() const {
    return m_orientationLock.has_value();
  }

  /**
   * Sets the drive subsystem to the specified chassis speeds.
   *
   * The speed direction must be relative to the "alliance wall".
   * Positive X is away from the wall.
   * Positive Y is to the left.
   * Zero heading is away from the wall.
   *
   * @param speeds Chassis speeds relative to the "alliance wall".
   */
  void SetChassisSpeeds(const frc::ChassisSpeeds& speeds);

  /**
   * Sets the drive motor controllers to the specified speeds and orientations.
   *
   * @param desiredStates Drive module states.
   * @param stopped Indicates that robot is stopped.
   */
  void SetModuleStates(
    wpi::array<SwerveModuleState, NUMBER_OF_MODULES> desiredStates, bool stopped = false);

  /**
   * Sets the maximum speed override.
   */
  void SetMaxSpeedOverride(units::meters_per_second_t speed) {
    m_maxSpeedOverride = speed;
  }

  /**
   * Removes the maximum speed override.
   */
  void ResetMaxSpeedOverride() {
    m_maxSpeedOverride = std::nullopt;
  }

  /**
   * Gets the beam vector to the loading zone vision target.
   */
  std::optional<frc::Pose2d> GetLoadBeam() const {
    // note that index here is zero-based
    return GetVisionTargetVector(
      frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed ? 4 : 3);
  }

  /**
   * Gets the vector to the vision target.
   */
  std::optional<frc::Pose2d> GetVisionTargetVector(int index) const {
    const auto target = m_visionTargets[index];
    return target.Y() >= 0_m ? std::optional<frc::Pose2d> { target } : std::nullopt;
  }

  /**
   * Gets the distance to the vision target.
   */
  std::optional<units::meter_t> GetVisionTargetDistance(int index) const {
    const auto translation = m_visionTargets[index].Translation();
    return translation.Y() >= 0_m ? std::optional<units::meter_t> { translation.Norm() }
                                  : std::nullopt;
  }

  /**
   * Gets the angle to the vision target.
   */
  std::optional<units::degree_t> GetVisionTargetAngle(int index) const {
    const auto target = m_visionTargets[index];
    return target.Y() >= 0_m ? std::optional<units::degree_t> { target.Rotation().Degrees() }
                             : std::nullopt;
  }

  /**
   * Kinematics describing the drive type.
   */
  SwerveDriveKinematics<NUMBER_OF_MODULES> kDriveKinematics {
    frc::Translation2d { DriveConstants::kWheelBase / 2, DriveConstants::kTrackWidth / 2 },
    frc::Translation2d { DriveConstants::kWheelBase / 2, -DriveConstants::kTrackWidth / 2 },
    frc::Translation2d { -DriveConstants::kWheelBase / 2, DriveConstants::kTrackWidth / 2 },
    frc::Translation2d { -DriveConstants::kWheelBase / 2, -DriveConstants::kTrackWidth / 2 }
  };

 private:
  /**
   * Gets the angle reading of the gyroscope.
   * @returns The angle in degrees.
   */
  units::degree_t GetGyroAngle() const {
    return units::degree_t { m_gyroSign * m_gyro.GetAngle() };
  }

  /**
   * Gets effective maximum speed.
   */
  units::meters_per_second_t GetMaxSpeed() const {
    return m_maxSpeedOverride.has_value() ? m_maxSpeedOverride.value() : m_maxSpeed;
  }

  frc::SlewRateLimiter<units::scalar> m_rotLimiter;
  frc::SlewRateLimiter<units::scalar> m_magLimiter;
  AngleSlewRateLimiter m_dirLimiter;

  double m_currentRotation = 0.0;
  double m_currentSpeed = 0.0;
  units::radian_t m_currentHeading = 0_rad;

  double m_inputRotation = 0.0;  // [-1, +1]
  double m_inputSpeed = 0.0;     // [-1,+ 1]
  units::radian_t m_inputHeading = 0_rad;

  units::meters_per_second_t m_currentVx;
  units::meters_per_second_t m_currentVy;
  units::radians_per_second_t m_currentOmega;

  double m_deadband = OIConstants::kDriveDeadband;
  units::meters_per_second_t m_maxSpeed = DriveConstants::kMaxSpeed;
  units::radians_per_second_t m_maxAngularSpeed = DriveConstants::kMaxAngularSpeed;

  std::optional<units::meters_per_second_t> m_maxSpeedOverride = std::nullopt;

  // constrain translational direction heading to a discrete step values (in radians)
  double m_headingStep = 0.0;

  // orientation to maintain while lock is in place
  std::optional<units::radian_t> m_orientationLock;
  frc::PIDController m_orientationController;

  // trust radius (must be within distance to the target for vision to be trusted)
  units::meter_t m_trustRadius = -1_m;
  units::meter_t m_tagLockX = 0_m;
  units::meter_t m_tagLockY = 0_m;
  units::second_t m_tagLockTime = 0_s;
  double m_trustVisionX = 1.0;
  double m_trustVisionY = 1.0;
  double m_trustVisionH = 100.0;

  // indicates that robot should maintain X wheel formation when stopped
  bool m_stopX;

  // components (e.g. motor controllers and sensors) should generally be
  // declared private and exposed only through public methods
  MAXSwerveModule m_frontLeft;
  MAXSwerveModule m_frontRight;
  MAXSwerveModule m_rearLeft;
  MAXSwerveModule m_rearRight;

  wpi::array<SwerveModuleState, NUMBER_OF_MODULES> m_desiredStates = {
    SwerveModuleState(),
    SwerveModuleState(),
    SwerveModuleState(),
    SwerveModuleState(),
  };

  TalonSRX m_gyroController { IntakeConstants::kIntakeLeaderControllerCanId };
  WPI_PigeonIMU m_gyro { m_gyroController };

  static constexpr double m_gyroSign = -1.0;  // flip the sign if changing gyro orientation
  static constexpr frc::Translation2d Zero = {};
  static constexpr frc::Pose2d NoPose = { -100_m, -100_m, 0_rad };

  units::radian_t m_pitch;
  units::radian_t m_roll;
  units::radians_per_second_t m_pitchVelocity;
  units::radians_per_second_t m_rollVelocity;

  // fused pitch and roll accounting for pose
  units::radian_t m_inclination;
  units::radians_per_second_t m_inclinationVelocity;

  // odometry tracking can be based on either XxxOdometry or XxxPoseEstimator
  // XxxOdometry relies only on encoders and gyro to estimate the pose
  // XxxPoseEstimator extends basic odometry tracking by accepting pose from
  // external vision system to compensate for the drift
  void UpdateOdometry();

  double GetVisionTargetDistanceTelemetry(int index) const {
    const auto d = GetVisionTargetDistance(index);
    return d.has_value() ? d.value().value() : 0.0;
  }

  double GetVisionTargetAngleTelemetry(int index) const {
    const auto d = GetVisionTargetAngle(index);
    return d.has_value() ? d.value().value() : -1.0;
  }

#ifdef DRIVETRAIN_ODOMETRY_VISION
  frc::SwerveDrivePoseEstimator<NUMBER_OF_MODULES> m_odometry;
#else
  frc::SwerveDriveOdometry<NUMBER_OF_MODULES> m_odometry;
#endif

  nt::DoubleArraySubscriber m_visionPoseSubscriber;
  nt::DoubleArraySubscriber m_visionTargetsSubscriber;

  // translation and direction to each of the vision targets
  frc::Pose2d m_visionTargets[8];

  // vision pose adjusted to the "alliance wall" coordinates
  frc::Pose2d m_visionPose;

  // identifier (index) of the currently locked target or -1 otherwise
  int m_lockedTargetId = -1;

  // timestamp when we had the lock acquired
  units::second_t m_lockedTargetTimestamp;

  // tracks robot position on the field (for debugging via Glass)
  frc::Field2d m_field;
  frc::Field2d m_fieldVision;
};
