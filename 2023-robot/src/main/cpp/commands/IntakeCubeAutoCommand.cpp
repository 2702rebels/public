// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeCubeAutoCommand.h"

IntakeCubeAutoCommand::IntakeCubeAutoCommand(IntakeSubsystem* intake) {
  SetName("IntakeCubeCommand");
  AddRequirements(intake);
  m_intake = intake;
}

// Called when the command is initially scheduled.
void IntakeCubeAutoCommand::Initialize() {
  m_intake->IntakeCube();
  m_intake->OpenAuto();
}

// Called repeatedly when this Command is scheduled to run
void IntakeCubeAutoCommand::Execute() {}

// Called once the command ends or is interrupted.
void IntakeCubeAutoCommand::End(bool interrupted) {
  m_intake->CloseCube();
  m_intake->HoldGamePiece();
}

// Returns true when the command should end.
bool IntakeCubeAutoCommand::IsFinished() {
#ifdef INTAKE_SWITCH_ENABLED
  return m_intake->IsGamePieceSwitchPressed();
#else
  return false;
#endif
}
