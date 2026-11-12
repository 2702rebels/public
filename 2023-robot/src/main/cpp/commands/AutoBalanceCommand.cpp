#include "commands/AutoBalanceCommand.h"

#include <frc/DataLogManager.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <units/math.h>

#include "Settings.h"
#include "utils/MathUtils.h"

// clang-format off
REGISTER_DOUBLE(AutoBalanceSpeed, "Auto/Balance/Speed", 1.1);
REGISTER_DOUBLE(AutoBalanceCrawlSpeed, "Auto/Balance/Crawl Speed", 0.35);
REGISTER_DOUBLE(AutoBalanceSpeedFactor, "Auto/Balance/Factor", 1.1);
REGISTER_DOUBLE(AutoBalanceRampTime, "Auto/Balance/Ramp Time", 1.2);
REGISTER_DOUBLE(AutoBalanceWaitTime, "Auto/Balance/Wait Time", 0.2);
REGISTER_DOUBLE(AutoBalanceMaxTime, "Auto/Balance/Max Time", 6.0);
REGISTER_DOUBLE(AutoBalanceAngleThreshold, "Auto/Balance/Angle Threshold", 7.0);
REGISTER_DOUBLE(AutoBalanceVelocityThreshold, "Auto/Balance/Velocity Threshold", 15.0);
REGISTER_DOUBLE(AutoBalanceLevelThreshold, "Auto/Balance/Level Threshold", 3.0);

// clang-format on

AutoBalanceCommand::AutoBalanceCommand(DriveSubsystem* drive, Side side) :
  m_drive { drive },
  m_direction { side == Side::INNER ? 1.0 : -1.0 } {
  AddRequirements(drive);
}

void AutoBalanceCommand::Initialize() {
  m_speed = units::meters_per_second_t { GetAutoBalanceSpeed() };
  m_factor = GetAutoBalanceSpeedFactor();
  m_rampTime = units::second_t { GetAutoBalanceRampTime() };
  m_waitTime = units::second_t { GetAutoBalanceWaitTime() };
  m_maxTime = units::second_t { GetAutoBalanceMaxTime() };
  m_angleThreshold = units::degree_t { GetAutoBalanceAngleThreshold() };
  m_levelThreshold = units::degree_t { GetAutoBalanceLevelThreshold() };
  m_velocityThreshold = units::degrees_per_second_t { GetAutoBalanceVelocityThreshold() };

  m_stage = Stage::APPROACH;
  m_angle = 0_rad;
  m_timer.Restart();
}

void AutoBalanceCommand::Execute() {
  const auto currentPose = m_drive->GetPose();

  m_angle = m_drive->GetInclination();
  m_velocity = m_drive->GetInclinationVelocity();

  auto speed = m_speed * m_direction;
  switch (m_stage) {
    case Stage::APPROACH:
      // start driving up the ramp
      if (units::math::fabs(m_angle) > m_angleThreshold) {
        m_stage = Stage::RAMP;
        m_timer.Restart();
      }
      break;

    case Stage::RAMP:
      if (m_timer.HasElapsed(m_rampTime)) {
        m_stage = Stage::BALANCE;
      }
      break;

    case Stage::BALANCE:
      // slow down as angle decreases
      speed = std::sqrt(units::math::fabs(m_angle).value()) * m_speed * m_direction * m_factor;

      if (
        units::math::fabs(m_angle) < m_angleThreshold ||
        (units::math::fabs(m_velocity) > m_velocityThreshold &&
         m_angle.value() * m_velocity.value() < 0)) {
        m_stage = Stage::WAIT;
        m_balancedAt = m_timer.Get();
      }
      break;

    case Stage::WAIT:
      speed = 0_mps;
      if (m_timer.Get() - m_balancedAt >= m_waitTime) {
        // if we are balanced -- end the command;
        // otherwise, slowly crawl to rebalance
        if (units::math::fabs(m_angle) < m_levelThreshold) {
          m_stage = Stage::FINAL;
        } else {
          speed = (m_angle > 0_rad ? 1_mps : -1_mps) * GetAutoBalanceCrawlSpeed();
        }
      } else {
        m_drive->StopX();
      }
      break;

    case Stage::FINAL:
      speed = 0_mps;
      break;
  }

  const auto speeds = frc::ChassisSpeeds::FromFieldRelativeSpeeds(
    speed, 0_mps, 0_rad_per_s, currentPose.Rotation().Radians());

  m_drive->SetChassisSpeeds(speeds);

#ifdef LOG_AUTO_COMMAND
  // clang-format off
  frc::SmartDashboard::PutNumber("Telemetry/AutoBalance/Speed", speed.value());
  frc::SmartDashboard::PutNumber("Telemetry/AutoBalance/Angle", m_angle.convert<units::degrees>().value());
  frc::SmartDashboard::PutNumber("Telemetry/AutoBalance/Angle V", m_velocity.convert<units::degrees_per_second>().value());
  frc::SmartDashboard::PutString("Telemetry/AutoBalance/Stage",
    m_stage == Stage::APPROACH ? "Approach" :
    m_stage == Stage::RAMP ? "Ramping" :
    m_stage == Stage::BALANCE ? "Balance" :
    m_stage == Stage::WAIT ? "Wait" : "Final");
  // clang-format on
#endif
}

void AutoBalanceCommand::End(bool interrupted) {
  m_drive->StopX();
  frc::SmartDashboard::PutString("Telemetry/AutoBalance/Stage", "Final");
}

bool AutoBalanceCommand::IsFinished() {
  return m_stage == Stage::FINAL || m_timer.HasElapsed(m_maxTime);
}
