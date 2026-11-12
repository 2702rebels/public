// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmHomeCommand.h"

#include <frc/Timer.h>

#include "commands/ArmHomeAxis1Command.h"
#include "commands/ArmHomeAxis2Command.h"
#include "commands/IntakeHomeGripperCommand.h"

ArmHomeCommand::ArmHomeCommand(ArmSubsystem* arm, IntakeSubsystem* intake) {
  SetName("ArmHomeCommand");
  AddCommands(IntakeHomeGripperCommand(intake), ArmHomeAxis2Command(arm), ArmHomeAxis1Command(arm));
}