#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/length.h>

#include "subsystems/DriveSubsystem.h"

class DriveOverBridgeCommand
  : public frc2::CommandHelper<frc2::CommandBase, DriveOverBridgeCommand> {
 public:
  DriveOverBridgeCommand(DriveSubsystem* drive);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  enum Stage {
    APPROACH,
    RAMP_UP,
    RAMP_DOWN,
    OUT,
    FINAL,
  };

  DriveSubsystem* m_drive;
  frc::Timer m_timer;
  Stage m_stage = Stage::APPROACH;
  units::radian_t m_angle;
  units::radian_t m_angleThreshold;
  units::radians_per_second_t m_velocity;
  units::radians_per_second_t m_velocityThreshold;
  units::meters_per_second_t m_speed;
  units::meters_per_second_t m_speedIn;
  units::meters_per_second_t m_speedOut;
  units::second_t m_timeOut;
};
