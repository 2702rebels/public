// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ArmSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ArmMoveCommand : public frc2::CommandHelper<frc2::CommandBase, ArmMoveCommand> {
 public:
  ArmMoveCommand(ArmSubsystem* arm, units::degree_t angle, units::inch_t distance);
  ArmMoveCommand(ArmSubsystem* arm, units::degree_t angle);
  ArmMoveCommand(ArmSubsystem* arm, units::inch_t distance);
  ArmMoveCommand(ArmSubsystem* arm, ArmPosition position);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ArmSubsystem* m_arm;
  units::degree_t m_axis1Angle;
  units::inch_t m_axis2Distance;
  ArmPosition m_position;
  bool m_isUsePosition = false;
  bool m_isAngleOnly = false;
  bool m_isDistanceOnly = false;
};
