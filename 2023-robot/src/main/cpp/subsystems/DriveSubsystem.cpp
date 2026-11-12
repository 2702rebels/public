#include "subsystems/DriveSubsystem.h"

#include <frc/DataLogManager.h>
#include <frc/MathUtil.h>
#include <frc/StateSpaceUtil.h>
#include <frc/controller/PIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <wpi/DataLog.h>
#include <wpi/sendable/SendableBuilder.h>

#include "Constants.h"
#include "Field.h"
#include "Settings.h"
#include "utils/MathUtils.h"
#include "utils/SwerveUtils.h"

using namespace DriveConstants;

// clang-format off
REGISTER_DOUBLE(DrivePoseStdDevStateX, "Drive/Pose/StdDev State/X", 0.1)
REGISTER_DOUBLE(DrivePoseStdDevStateY, "Drive/Pose/StdDev State/Y", 0.1)
REGISTER_DOUBLE(DrivePoseStdDevStateH, "Drive/Pose/StdDev State/θ", 0.1)
REGISTER_DOUBLE(DrivePoseStdDevVisionX, "Drive/Pose/StdDev Vision/X", 0.1)
REGISTER_DOUBLE(DrivePoseStdDevVisionY, "Drive/Pose/StdDev Vision/Y", 0.1)
REGISTER_DOUBLE(DrivePoseStdDevVisionH, "Drive/Pose/StdDev Vision/θ", 0.1)
REGISTER_DOUBLE(DriveSlewRateDirection, "Drive/Slew/Direction", DriveConstants::kDirectionSlewRate)
REGISTER_DOUBLE(DriveSlewRateAccelerate, "Drive/Slew/Accelerate", DriveConstants::kAccelerateSlewRate)
REGISTER_DOUBLE(DriveSlewRateDecelerate, "Drive/Slew/Decelerate", DriveConstants::kDecelerateSlewRate)
REGISTER_DOUBLE(DriveSlewRateRotation, "Drive/Slew/Rotation", DriveConstants::kRotationalSlewRate)
REGISTER_DOUBLE(DriveTuneHeadingStep, "Drive/Tune/Heading Step", 5)
REGISTER_DOUBLE(DriveTuneMaxSpeed, "Drive/Tune/Max Speed (mps)", DriveConstants::kMaxSpeed.value())
REGISTER_DOUBLE(DriveTuneMaxAngularSpeed, "Drive/Tune/Max Angular Speed (dps)", DriveConstants::kMaxAngularSpeed.convert<units::degrees_per_second>().value())
REGISTER_DOUBLE(DriveTuneDeadband, "Drive/Tune/Deadband", OIConstants::kDriveDeadband)
REGISTER_DOUBLE(DriveOrientationP, "Drive/Tune/Orientation kP", 0.5)
REGISTER_DOUBLE(DriveVisionTrustRadius, "Drive/Vision/Trust Radius", 3)
REGISTER_DOUBLE(DriveVisionTagLockX, "Drive/Vision/Tag Lock X", 0.1)
REGISTER_DOUBLE(DriveVisionTagLockY, "Drive/Vision/Tag Lock Y", 1)
REGISTER_DOUBLE(DriveVisionTagLockTime, "Drive/Vision/Tag Lock Time (s)", 2)

// clang-format on

double to180(units::radian_t value) {
  return MathUtils::ClampAngle(value).convert<units::degrees>().value();
}

double to90(units::degree_t value) {
  const auto v = fabs(value.value());
  return v - ((int) v / 90.0) * 90.0;  // modulus 90
}

static const double defaultVisionPosition[] = { -1.0, -1.0, 0.0 };
static const double defaultVisionTargets[] = {
  -1.0, -1.0, 0.0, -1.0, -1.0, 0.0, -1.0, -1.0, 0.0, -1.0, -1.0, 0.0,
  -1.0, -1.0, 0.0, -1.0, -1.0, 0.0, -1.0, -1.0, 0.0, -1.0, -1.0, 0.0,
};

DriveSubsystem::DriveSubsystem() :
  m_rotLimiter(GetDriveSlewRateRotation() / 1_s),
  m_magLimiter(GetDriveSlewRateAccelerate() / 1_s, -fabs(GetDriveSlewRateDecelerate()) / 1_s),
  m_dirLimiter { units::radian_t { GetDriveSlewRateDirection() } / 1_s },
  m_deadband { GetDriveTuneDeadband() },
  m_maxSpeed { units::meters_per_second_t { GetDriveTuneMaxSpeed() } },
  m_maxAngularSpeed { units::degrees_per_second_t { GetDriveTuneMaxAngularSpeed() } },
  m_headingStep { units::convert<units::degrees, units::radians>(GetDriveTuneHeadingStep()) },
  m_orientationController { GetDriveOrientationP(), 0.0, 0.0 },
  m_trustRadius { units::meter_t { GetDriveVisionTrustRadius() } },
  m_tagLockX { units::meter_t { GetDriveVisionTagLockX() } },
  m_tagLockY { units::meter_t { GetDriveVisionTagLockY() } },
  m_tagLockTime { units::second_t { GetDriveVisionTagLockTime() } },
  m_trustVisionX { GetDrivePoseStdDevVisionX() },
  m_trustVisionY { GetDrivePoseStdDevVisionY() },
  m_trustVisionH { GetDrivePoseStdDevVisionH() },
  m_frontLeft {
    kFrontLeftDrivingCanId,
    kFrontLeftTurningCanId,
    kFrontLeftChassisAngularOffset,
  },
  m_frontRight {
    kFrontRightDrivingCanId,
    kFrontRightTurningCanId,
    kFrontRightChassisAngularOffset,
  },
  m_rearLeft {
    kRearLeftDrivingCanId,
    kRearLeftTurningCanId,
    kRearLeftChassisAngularOffset,
  },
  m_rearRight {
    kRearRightDrivingCanId,
    kRearRightTurningCanId,
    kRearRightChassisAngularOffset,
  },
  m_odometry {
    kDriveKinematics,
    GetYaw(),
    {
      m_frontLeft.GetPosition(),
      m_frontRight.GetPosition(),
      m_rearLeft.GetPosition(),
      m_rearRight.GetPosition(),
    },
    frc::Pose2d {},
#ifdef DRIVETRAIN_ODOMETRY_VISION
    { GetDrivePoseStdDevStateX(), GetDrivePoseStdDevStateY(), GetDrivePoseStdDevStateH() },
    { GetDrivePoseStdDevVisionX(), GetDrivePoseStdDevVisionY(), GetDrivePoseStdDevVisionH() },
#endif
  } {
  m_orientationController.EnableContinuousInput(-std::numbers::pi, std::numbers::pi);

  frc::SmartDashboard::PutData("Field", &m_field);
  frc::SmartDashboard::PutData("Field Vision", &m_fieldVision);
  frc::SmartDashboard::PutData("Gyro", &m_gyro);

  const auto ntinst = nt::NetworkTableInstance::GetDefault();
  m_visionPoseSubscriber =
    ntinst.GetDoubleArrayTopic("/Vision/PoseArray").Subscribe(defaultVisionPosition);
  m_visionTargetsSubscriber =
    ntinst.GetDoubleArrayTopic("/Vision/TargetVectors").Subscribe(defaultVisionTargets);
}

void DriveSubsystem::InitSendable(wpi::SendableBuilder& builder) {
  SubsystemBase::InitSendable(builder);
  SubsystemBase::SetName("Drive");
  SubsystemBase::SetSubsystem("Drive");

  builder.SetSmartDashboardType("Drive");

  // clang-format off
  // user input from joystick/controller and values commanded to the modules
  builder.AddDoubleProperty("Telemetry/Joystick/Heading",   [this] { return m_inputHeading.convert<units::degrees>().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Joystick/Magnitude", [this] { return m_inputSpeed; }, nullptr);
  builder.AddDoubleProperty("Telemetry/Joystick/Rotation",  [this] { return m_inputRotation; }, nullptr);
  builder.AddDoubleProperty("Telemetry/Commanded/Heading",  [this] { return m_currentHeading.convert<units::degrees>().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Commanded/Speed",    [this] { return m_currentSpeed; }, nullptr);
  builder.AddDoubleProperty("Telemetry/Commanded/Rotation", [this] { return m_currentRotation; }, nullptr);

  // real-time telemetry of the robot speeds, heading, inclination, etc.
  builder.AddDoubleProperty("Telemetry/Pose/X",             [this] { return GetPose().X().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Pose/Y",             [this] { return GetPose().Y().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Pose/Θ",             [this] { return to180(GetPose().Rotation().Radians()); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision Pose/X",      [this] { return GetVisionPose().X().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision Pose/Y",      [this] { return GetVisionPose().Y().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision Pose/Θ",      [this] { return to180(GetVisionPose().Rotation().Radians()); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Speed/Vx",           [this] { return m_currentVx.value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Speed/Vy",           [this] { return m_currentVy.value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Speed/ω",            [this] { return m_currentOmega.convert<units::deg_per_s>().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Gyro/Yaw",           [this] { return to180(GetYaw().Radians()); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Gyro/Pitch",         [this] { return GetPitch().convert<units::degrees>().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Gyro/Roll",          [this] { return GetRoll().convert<units::degrees>().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Gyro/Pitch per s",   [this] { return GetPitchVelocity().convert<units::deg_per_s>().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Gyro/Roll per s",    [this] { return GetRollVelocity().convert<units::deg_per_s>().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Gyro/Inclination",   [this] { return GetInclination().convert<units::degree>().value(); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Gyro/Incl per s",    [this] { return GetInclinationVelocity().convert<units::deg_per_s>().value(); }, nullptr);

  // vision
  #ifdef LOG_VISION_TARGETS
  builder.AddDoubleProperty("Telemetry/Vision/T1/L",        [this] { return GetVisionTargetDistanceTelemetry(0); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T2/L",        [this] { return GetVisionTargetDistanceTelemetry(1); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T3/L",        [this] { return GetVisionTargetDistanceTelemetry(2); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T4/L",        [this] { return GetVisionTargetDistanceTelemetry(3); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T5/L",        [this] { return GetVisionTargetDistanceTelemetry(4); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T6/L",        [this] { return GetVisionTargetDistanceTelemetry(5); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T7/L",        [this] { return GetVisionTargetDistanceTelemetry(6); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T8/L",        [this] { return GetVisionTargetDistanceTelemetry(7); }, nullptr);

  builder.AddDoubleProperty("Telemetry/Vision/T1/Θ",        [this] { return GetVisionTargetAngleTelemetry(0); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T2/Θ",        [this] { return GetVisionTargetAngleTelemetry(1); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T3/Θ",        [this] { return GetVisionTargetAngleTelemetry(2); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T4/Θ",        [this] { return GetVisionTargetAngleTelemetry(3); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T5/Θ",        [this] { return GetVisionTargetAngleTelemetry(4); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T6/Θ",        [this] { return GetVisionTargetAngleTelemetry(5); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T7/Θ",        [this] { return GetVisionTargetAngleTelemetry(6); }, nullptr);
  builder.AddDoubleProperty("Telemetry/Vision/T8/Θ",        [this] { return GetVisionTargetAngleTelemetry(7); }, nullptr);
  #endif

  // states
  builder.AddBooleanProperty("Telemetry/X formation",       [this] { return m_stopX; }, nullptr);
  builder.AddBooleanProperty("Telemetry/IMU Ready",         [this] { return m_gyro.GetState() == ctre::phoenix::sensors::PigeonIMU::PigeonState::Ready; }, nullptr);
  builder.AddBooleanProperty("Telemetry/Orientation Lock",  [this] { return HasOrientationLock(); }, nullptr);
  // clang-format on

  // swerve module states
  builder.AddDoubleProperty(
    "Telemetry/Swerve States/Driving Current",
    [this] {
      return std::max(
        std::max(m_frontLeft.GetDrivingCurrent(), m_frontRight.GetDrivingCurrent()),
        std::max(m_rearLeft.GetDrivingCurrent(), m_rearRight.GetDrivingCurrent()));
    },
    nullptr);
  builder.AddDoubleProperty(
    "Telemetry/Swerve States/Turning Current",
    [this] {
      return std::max(
        std::max(m_frontLeft.GetTurningCurrent(), m_frontRight.GetTurningCurrent()),
        std::max(m_rearLeft.GetTurningCurrent(), m_rearRight.GetTurningCurrent()));
    },
    nullptr);
  builder.AddDoubleArrayProperty(
    "Telemetry/Swerve States/Measured",
    [this] {
      return std::vector<double> {
        to90(m_frontLeft.GetState().angle.Degrees()),  fabs(m_frontLeft.GetState().speed.value()),
        to90(m_frontRight.GetState().angle.Degrees()), fabs(m_frontRight.GetState().speed.value()),
        to90(m_rearLeft.GetState().angle.Degrees()),   fabs(m_rearLeft.GetState().speed.value()),
        to90(m_rearRight.GetState().angle.Degrees()),  fabs(m_rearRight.GetState().speed.value()),
      };
    },
    nullptr);
  builder.AddDoubleArrayProperty(
    "Telemetry/Swerve States/Desired",
    [this] {
      return std::vector<double> {
        to90(m_frontLeft.GetDesiredAngle()),  fabs(m_frontLeft.GetDesiredSpeed().value()),
        to90(m_frontRight.GetDesiredAngle()), fabs(m_frontRight.GetDesiredSpeed().value()),
        to90(m_rearLeft.GetDesiredAngle()),   fabs(m_rearLeft.GetDesiredSpeed().value()),
        to90(m_rearRight.GetDesiredAngle()),  fabs(m_rearRight.GetDesiredSpeed().value()),
      };
    },
    nullptr);
}

void DriveSubsystem::Periodic() {
  UpdateOdometry();
}

void DriveSubsystem::Drive(double xSpeed, double ySpeed, double rotSpeed, bool fieldRelative) {
  if (rotSpeed != 0 && m_orientationLock.has_value()) {
    DisableOrientationLock();
  }

  // convert XY to polar for rate limiting

  // clamp vector to unit circle, since joystick reports values that
  // overshoot slightly when not aligned with X or Y axis
  m_inputSpeed = std::clamp(sqrt(pow(xSpeed, 2) + pow(ySpeed, 2)), -1.0, 1.0);

  // maintain last heading at stop / very low speeds to prevent
  // abrupt changes to lateral direction that causes the robot
  // to jerk when the joystick is released
  if (!MathUtils::IsZero(m_inputSpeed, 0.01)) {
    m_inputHeading = MathUtils::ClampAngle(units::radian_t {
      MathUtils::RoundToStep(atan2(ySpeed, xSpeed), m_headingStep),
    });
  }

  m_inputRotation = rotSpeed;

  // reset X wheel formation when input is received
  if (m_stopX && (!MathUtils::IsZero(m_inputRotation) || !MathUtils::IsZero(m_inputSpeed))) {
    m_stopX = false;
  }

  // apply rate limiting to rotation
  m_currentRotation = m_rotLimiter.Calculate(m_inputRotation);

  // use PID for orientation
  if (m_orientationLock.has_value()) {
    m_currentRotation = m_orientationController.Calculate(GetHeading().Radians().value());
  }

  // make heading slew rate dependent on the magnitude of lateral "speed"
  // when the robot is moving slowly we should allow for much faster changes
  // to the directional heading, while clamping it down at higher speeds
  //
  // here we calculate a "damping" factor as the inverse of the lateral "speed";
  // at maximum "speed" (1) the factor is 1, forcing the limiter to apply
  // the maximum constraints; as the speed decreases the factor increases
  // effectively increasing the maximum allowed slew rate
  const auto factor = MathUtils::IsZero(m_currentSpeed) ? 1000.0 : fabs(1.0 / m_currentSpeed);

  // we consider three scenarios based on the angular sectors where
  // new heading falls in relative to the current heading (last commanded):
  // 1. the 80° sector in front
  // 2. the 80° sector in rear
  // 3. the 100° left and right sectors
  //
  // - in the first case we continue maintaining commanded speed and steer
  // as usual (subject to dumping factor);
  // - in the second case we keep current heading while slowing down to
  // a full stop then flip the heading 180° and start accelerating to
  // the commanded speed (places us back to case 1.)
  // - in the third case we start slowing down to a full stop while steering
  // towards new heading until we are back to the case 1.

  const auto headingDelta =
    units::math::fabs(MathUtils::ClampAngle(m_inputHeading - m_currentHeading));

  constexpr auto sectorF = units::radian_t { 0.22 * std::numbers::pi };
  constexpr auto sectorR = units::radian_t { 0.78 * std::numbers::pi };

  if (headingDelta < sectorF) {
    m_currentSpeed = m_magLimiter.Calculate(m_inputSpeed);
    m_currentHeading = m_dirLimiter.Calculate(m_inputHeading, factor);
  } else if (headingDelta > sectorR) {
    if (!MathUtils::IsZero(m_currentSpeed)) {
      // keep heading
      m_currentSpeed = m_magLimiter.Calculate(0.0);
    } else {
      m_currentSpeed = m_magLimiter.Calculate(m_inputSpeed);
      m_currentHeading =
        MathUtils::ClampAngle(m_currentHeading + units::radian_t { std::numbers::pi });
    }
  } else {
    m_currentSpeed = m_magLimiter.Calculate(0.0);
    m_currentHeading = m_dirLimiter.Calculate(m_inputHeading, factor);
  }

  const auto xSpeedCommanded = m_currentSpeed * units::math::cos(m_currentHeading);
  const auto ySpeedCommanded = m_currentSpeed * units::math::sin(m_currentHeading);

  // convert the commanded speeds into the correct units for the drivetrain
  const auto maxSpeed = GetMaxSpeed();
  const auto xSpeedDelivered = xSpeedCommanded * maxSpeed;
  const auto ySpeedDelivered = ySpeedCommanded * maxSpeed;
  const auto rotDelivered = m_currentRotation * m_maxAngularSpeed;

  const auto speeds = fieldRelative
                      ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                          xSpeedDelivered, ySpeedDelivered, rotDelivered, GetHeading())
                      : frc::ChassisSpeeds { xSpeedDelivered, ySpeedDelivered, rotDelivered };

  SetChassisSpeeds(speeds);
}

void DriveSubsystem::StopX() {
  m_stopX = true;
  Stop();
}

void DriveSubsystem::SetChassisSpeeds(const frc::ChassisSpeeds& speeds) {
  m_currentVx = speeds.vx;
  m_currentVy = speeds.vy;
  m_currentOmega = speeds.omega;

#ifdef SWERVE_SECOND_ORDER_KINEMATICS
  const auto states = kDriveKinematics.ToSwerveModuleStates2(speeds);
#else
  const auto states = kDriveKinematics.ToSwerveModuleStates(speeds);
#endif
  const auto stopped = MathUtils::IsZero(speeds.vx.value()) &&
                       MathUtils::IsZero(speeds.vy.value()) &&
                       MathUtils::IsZero(speeds.omega.value());

  SetModuleStates(states, stopped);
}

void DriveSubsystem::SetModuleStates(
  wpi::array<SwerveModuleState, NUMBER_OF_MODULES> desiredStates, bool stopped) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates, GetMaxSpeed());

  // X wheel formation when stopped
  if (stopped && m_stopX) {
    desiredStates[0] = SwerveModuleState { 0_mps, frc::Rotation2d { 45_deg } };
    desiredStates[1] = SwerveModuleState { 0_mps, frc::Rotation2d { -45_deg } };
    desiredStates[2] = SwerveModuleState { 0_mps, frc::Rotation2d { -45_deg } };
    desiredStates[3] = SwerveModuleState { 0_mps, frc::Rotation2d { 45_deg } };
  }

  m_desiredStates = desiredStates;
  const auto [fl, fr, rl, rr] = desiredStates;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(rl);
  m_rearRight.SetDesiredState(rr);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_rearRight.ResetEncoders();
}

void DriveSubsystem::ZeroHeading() {
  const auto pose = GetPose();
  m_odometry.ResetPosition(
    GetYaw(),
    {
      m_frontLeft.GetPosition(),
      m_frontRight.GetPosition(),
      m_rearLeft.GetPosition(),
      m_rearRight.GetPosition(),
    },
    {
      pose.X(),
      pose.Y(),
      0_rad,
    });
}

void DriveSubsystem::UpdateOdometry() {
  m_odometry.Update(
    GetYaw(),
    {
      m_frontLeft.GetPosition(),
      m_frontRight.GetPosition(),
      m_rearLeft.GetPosition(),
      m_rearRight.GetPosition(),
    });

  const auto visionTargets = m_visionTargetsSubscriber.GetAtomic();

  // update vision target vectors and find the nearest one based on the Euclidean distance
  units::meter_t shortestDistance = 999_m;  // some big number
  int nearestTargetId = -1;

  for (int i = 0; i < 8; ++i) {
    m_visionTargets[i] = frc::Pose2d(
      units::meter_t { visionTargets.value[i * 3] },
      units::meter_t { visionTargets.value[i * 3 + 1] },
      units::radian_t { visionTargets.value[i * 3 + 2] });

    // important: vision target vector is pointing from the robot's center to the tag
    // X indicates lateral translation (left or right) and can be negative
    // Y indicates distance (projection) to the tag's plane and is always positive
    //
    // negative Y value is an indicator that tag's vector is not reported
    if (m_visionTargets[i].Y() >= 0_m) {
      const auto distance = m_visionTargets[i].Translation().Norm();
      if (distance < shortestDistance) {
        shortestDistance = distance;
        nearestTargetId = i;
      }
    }
  }

  // we consider target "locked" when we are close (within 1 meter of
  // the tag's Y coordinate of the vector) and we are aligned against
  // the tag within certain boundaries (e.g. +/- 10cm)
  if (nearestTargetId >= 0) {
    const auto x = m_visionTargets[nearestTargetId].X();
    const auto y = m_visionTargets[nearestTargetId].Y();

    if (units::math::fabs(x) < m_tagLockX && units::math::fabs(y) < m_tagLockY) {
      if (m_lockedTargetId != nearestTargetId) {
        m_lockedTargetId = nearestTargetId;
        m_lockedTargetTimestamp = frc::Timer::GetFPGATimestamp();
      }
    }
  } else {
    // reset any previous locks
    m_lockedTargetId = -1;
    m_lockedTargetTimestamp = -1_s;
  }

#ifdef DRIVETRAIN_ODOMETRY_VISION
  // code here uses wpilib odometry that employs Kalman filters
  // to compensate for noisy data coming from vision;
  // however, it was discovered that produced estimated position
  // swings dramatically (and inexplicably) after we restart
  // feeding vision data once tag has been required, making
  // this approach practically unusable

  // feed global position from camera
  const auto visionPose = m_visionPoseSubscriber.GetAtomic();
  const auto x = units::meter_t { visionPose.value[0] };

  // only use pose if we have one and if are within trust radius of any target
  if (x.value() >= 0.0 && shortestDistance > 0_m && shortestDistance < m_trustRadius) {
    const auto timestamp = units::microsecond_t { (double) visionPose.time };
    const auto y = units::meter_t { visionPose.value[1] };
    const auto r = units::radian_t { visionPose.value[2] };

    m_visionPose =
      Field::ToAlliancePose(frc::Pose2d { frc::Translation2d(x, y), frc::Rotation2d(r) });

    m_odometry.SetVisionMeasurementStdDevs({
      shortestDistance.value() * m_trustVisionX,
      shortestDistance.value() * m_trustVisionY,
      m_trustVisionH,
    });

    m_odometry.AddVisionMeasurement(m_visionPose, timestamp);
  } else {
    m_visionPose = NoPose;
  }
#else
  // code here uses vision to reset odometry position, but not orientation
  // unless we have a good target lock for some time;
  // this allows periodic reset of the odometry without compromising driver's
  // ability to control the robot, that depends exclusively on the correct
  // orientation of the robot; essentially we put our trust in the gyro to
  // control the orientation but periodically correct field positioning;
  //
  // worst case scenario, when the field placement is invalid, will skew
  // auto alignment until positioning is restored, but won't tamper with
  // driver's teleop controls

  const auto visionPose = m_visionPoseSubscriber.GetAtomic();
  const auto x = units::meter_t { visionPose.value[0] };

  // X coordinate would be negative when the pose is not reported;
  // vision code does some filtering of bad poses and won't report poses
  // that fall outside of its trust radius (~3m)
  if (x.value() >= 0.0) {
    // const auto timestamp = units::microsecond_t { (double) visionPose.time };
    const auto y = units::meter_t { visionPose.value[1] };
    const auto r = units::radian_t { visionPose.value[2] };

    m_visionPose =
      Field::ToAlliancePose(frc::Pose2d { frc::Translation2d(x, y), frc::Rotation2d(r) });

    // if do we have lock acquired long enough, use orientation from vision
    // otherwise, keep use current heading from the gyro
    const auto currentTime = frc::Timer::GetFPGATimestamp();
    // const auto useVisionHeading = m_lockedTargetId >= 0 && m_tagLockTime > 0_s &&
    //                               currentTime - m_lockedTargetTimestamp > m_tagLockTime;

    const auto useVisionHeading = false;

    m_odometry.ResetPosition(
      GetYaw(),
      {
        m_frontLeft.GetPosition(),
        m_frontRight.GetPosition(),
        m_rearLeft.GetPosition(),
        m_rearRight.GetPosition(),
      },
      frc::Pose2d {
        m_visionPose.Translation(),
        useVisionHeading ? m_visionPose.Rotation() : GetHeading(),
      });
  }
#endif

  // update field position for debugging purposes
  const auto pose = GetPose();
  m_field.SetRobotPose(pose);
  m_fieldVision.SetRobotPose(m_visionPose);

  // update pitch/roll/inclination
  const auto heading = pose.Rotation();

  // gyro reading
  double ypr[3];
  double xyz[3];
  m_gyro.GetYawPitchRoll(ypr);
  m_gyro.GetRawGyro(xyz);

  m_pitch = units::degree_t { ypr[1] };
  m_roll = units::degree_t { ypr[2] };

  m_pitchVelocity = units::degrees_per_second_t { xyz[0] };
  m_rollVelocity = units::degrees_per_second_t { xyz[1] };

  const auto cosTheta = heading.Cos();
  const auto sinTheta = heading.Sin();

  m_inclination = cosTheta * m_pitch.convert<units::radians>() +
                  m_gyroSign * sinTheta * m_roll.convert<units::radians>();

  m_inclinationVelocity =
    cosTheta * m_pitchVelocity.convert<units::radians_per_second>() +
    m_gyroSign * sinTheta * m_rollVelocity.convert<units::radians_per_second>();
}

frc::Pose2d DriveSubsystem::GetPose() const {
#ifdef DRIVETRAIN_ODOMETRY_VISION
  return m_odometry.GetEstimatedPosition();
#else
  return m_odometry.GetPose();
#endif
}

void DriveSubsystem::UpdateTuningParameters() {
  frc::DataLogManager::Log("Drive: Retuning");

  m_deadband = GetDriveTuneDeadband();
  m_maxSpeed = units::meters_per_second_t { GetDriveTuneMaxSpeed() };
  m_maxAngularSpeed = units::degrees_per_second_t { GetDriveTuneMaxAngularSpeed() };
  m_headingStep = units::convert<units::degrees, units::radians>(GetDriveTuneHeadingStep());
  m_trustRadius = units::meter_t { GetDriveVisionTrustRadius() };
  m_tagLockX = { units::meter_t { GetDriveVisionTagLockX() } };
  m_tagLockY = { units::meter_t { GetDriveVisionTagLockY() } };
  m_tagLockTime = { units::second_t { GetDriveVisionTagLockTime() } };
  m_trustVisionX = GetDrivePoseStdDevVisionX();
  m_trustVisionY = GetDrivePoseStdDevVisionY();
  m_trustVisionH = GetDrivePoseStdDevVisionH();

  // re-create slew rate limiters for translation magnitude and rotation
  // since these do not have a mechanism to update rate limits at runtime
  m_rotLimiter = frc::SlewRateLimiter<units::scalar>(GetDriveSlewRateRotation() / 1_s);
  m_magLimiter = frc::SlewRateLimiter<units::scalar>(
    GetDriveSlewRateAccelerate() / 1_s, -fabs(GetDriveSlewRateDecelerate()) / 1_s);
  m_dirLimiter.SetLimit(units::radian_t { GetDriveSlewRateDirection() } / 1_s);

  // update modules
  m_frontLeft.UpdateTuningParameters();
  m_frontRight.UpdateTuningParameters();
  m_rearLeft.UpdateTuningParameters();
  m_rearRight.UpdateTuningParameters();

  m_orientationController.SetP(GetDriveOrientationP());
}

void DriveSubsystem::ResetOdometry(const frc::Pose2d& pose) {
  m_odometry.ResetPosition(
    GetYaw(),
    {
      m_frontLeft.GetPosition(),
      m_frontRight.GetPosition(),
      m_rearLeft.GetPosition(),
      m_rearRight.GetPosition(),
    },
    pose);
}

void DriveSubsystem::EnableOrientationLock(units::radian_t angle) {
  m_orientationLock = angle;
  m_orientationController.SetSetpoint(angle.convert<units::radians>().value());
}

void DriveSubsystem::ToggleOrientationLock(units::radian_t angle) {
  if (m_orientationLock.has_value() && m_orientationLock.value() == angle) {
    DisableOrientationLock();
  } else {
    EnableOrientationLock(angle);
  }
}

void DriveSubsystem::DisableOrientationLock() {
  m_orientationLock = std::nullopt;
  m_orientationController.SetSetpoint(0);
}

void DriveSubsystem::Stop() {
  Drive(0, 0, 0, true);
}
