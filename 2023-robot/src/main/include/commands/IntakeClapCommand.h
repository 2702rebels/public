#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/IntakeSubsystem.h"

class IntakeClapCommand : public frc2::CommandHelper<frc2::CommandBase, IntakeClapCommand> {
 public:
  IntakeClapCommand(IntakeSubsystem* m_intake);

  void Initialize() override;

  void Execute() override;

  void End(bool interrupted) override;

  bool IsFinished() override;
};
