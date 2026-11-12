#include "commands/DriveOverBridgeCommand.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/angular_velocity.h>
#include <units/math.h>
#include <units/velocity.h>

#include "Settings.h"
#include "utils/MathUtils.h"

// clang-format off
REGISTER_DOUBLE(DriveOverBridgeCommandSpeedIn, "Auto/DriveOverBridge/Speed In", 1.2);
REGISTER_DOUBLE(DriveOverBridgeCommandSpeedOut, "Auto/DriveOverBridge/Speed Out", 0.7);
REGISTER_DOUBLE(DriveOverBridgeCommandAngleThreshold, "Auto/DriveOverBridge/Angle Threshold", 8.0);
REGISTER_DOUBLE(DriveOverBridgeCommandVelocityThreshold, "Auto/DriveOverBridge/Velocity Threshold", 40.0);
REGISTER_DOUBLE(DriveOverBridgeCommandTimeOut, "Auto/DriveOverBridge/Time Out", 0.5);

// clang-format on

DriveOverBridgeCommand::DriveOverBridgeCommand(DriveSubsystem* drive) : m_drive { drive } {
  AddRequirements(drive);
}

void DriveOverBridgeCommand::Initialize() {
  m_speedIn = units::meters_per_second_t { GetDriveOverBridgeCommandSpeedIn() };
  m_speedOut = units::meters_per_second_t { GetDriveOverBridgeCommandSpeedOut() };
  m_angleThreshold = units::degree_t { GetDriveOverBridgeCommandAngleThreshold() };
  m_velocityThreshold =
    units::degrees_per_second_t { GetDriveOverBridgeCommandVelocityThreshold() };
  m_timeOut = units::second_t { GetDriveOverBridgeCommandTimeOut() };

  m_speed = m_speedIn;
  m_stage = Stage::APPROACH;
  m_angle = 0_rad;
  m_timer.Reset();
}

void DriveOverBridgeCommand::Execute() {
  const auto currentPose = m_drive->GetPose();

  m_angle = m_drive->GetInclination();
  m_velocity = m_drive->GetInclinationVelocity();

  switch (m_stage) {
    case Stage::APPROACH:
      // start driving up the ramp
      if (m_angle > m_angleThreshold) {
        m_stage = Stage::RAMP_UP;
      }
      break;

    case Stage::RAMP_UP:
      // reduce speed as we start going downhill
      if (m_velocity < -m_velocityThreshold) {
        m_speed = m_speedOut;
      }

      // inclination becomes negative -- start driving down the ramp
      if (m_angle < -m_angleThreshold) {
        m_stage = Stage::RAMP_DOWN;
      }
      break;

    case Stage::RAMP_DOWN:
      // wait until we are on the ground -- start timer to reverse
      if (units::math::fabs(m_angle) < 2_deg) {
        m_stage = Stage::OUT;
        m_timer.Restart();
      }
      break;

    case Stage::OUT:
    case Stage::FINAL:
      break;
  }

  const auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    m_speed, 0_mps, 0_rad_per_s, currentPose.Rotation().Radians());

  m_drive->SetChassisSpeeds(speeds);

#ifdef LOG_AUTO_COMMAND
  // clang-format off
  frc::SmartDashboard::PutNumber("Telemetry/DriveOverBridge/Angle", m_angle.convert<units::degrees>().value());
  frc::SmartDashboard::PutNumber("Telemetry/DriveOverBridge/Velocity", m_velocity.convert<units::degrees_per_second>().value());
  frc::SmartDashboard::PutString("Telemetry/DriveOverBridge/Stage",
    m_stage == Stage::APPROACH ? "Approach":
    m_stage == Stage::RAMP_UP ? "Ramp Up" :
    m_stage == Stage::RAMP_DOWN ? "Ramp Down" :
    m_stage == Stage::OUT ? "Out" : "Final");
  // clang-format on
#endif
}

void DriveOverBridgeCommand::End(bool interrupted) {
  m_drive->Stop();
  frc::SmartDashboard::PutString(
    "Telemetry/DriveOverBridge/Stage", interrupted ? "Final (interrupted)" : "Final");
}

bool DriveOverBridgeCommand::IsFinished() {
  return m_stage == Stage::OUT && m_timer.HasElapsed(m_timeOut);
}
