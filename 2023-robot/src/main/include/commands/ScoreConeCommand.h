// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/ArmSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

/**
 * An example command.
 *
 * <p>Note that this extends CommandHelper, rather extending CommandBase
 * directly; this is crucially important, or else the decorator functions in
 * Command will *not* work!
 */
class ScoreConeCommand : public frc2::CommandHelper<frc2::CommandBase, ScoreConeCommand> {
 public:
  ScoreConeCommand(ArmSubsystem* arm, IntakeSubsystem* intake);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;

 private:
  ArmSubsystem* m_arm;
  IntakeSubsystem* m_intake;
  bool m_armisClear = false;
  units::millisecond_t m_outtakingStartTime;
};
