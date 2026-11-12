#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/length.h>
#include <units/velocity.h>

#include "subsystems/DriveSubsystem.h"

class AutoBalanceCommand : public frc2::CommandHelper<frc2::CommandBase, AutoBalanceCommand> {
 public:
  AutoBalanceCommand(DriveSubsystem* drive, Side side = Side::INNER);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  enum Stage {
    APPROACH,
    RAMP,
    BALANCE,
    WAIT,
    FINAL,
  };

  DriveSubsystem* m_drive;
  frc::Timer m_timer;
  Stage m_stage = Stage::APPROACH;
  double m_direction = 1.0;
  double m_factor = 1.0;
  units::second_t m_rampTime;
  units::second_t m_waitTime;
  units::second_t m_maxTime;
  units::second_t m_balancedAt;
  units::radian_t m_angle;
  units::radian_t m_angleThreshold;
  units::radian_t m_levelThreshold;
  units::radians_per_second_t m_velocity;
  units::radians_per_second_t m_velocityThreshold;
  units::meters_per_second_t m_speed;
};
