// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeConeCommand.h"

IntakeConeCommand::IntakeConeCommand(IntakeSubsystem* intake) {
  SetName("IntakeConeCommand");
  AddRequirements(intake);
  m_intake = intake;
}

// Called when the command is initially scheduled.
void IntakeConeCommand::Initialize() {
  m_intake->IntakeCone();
  m_intake->OpenCone();
}

// Called repeatedly when this Command is scheduled to run
void IntakeConeCommand::Execute() {}

// Called once the command ends or is interrupted.
void IntakeConeCommand::End(bool interrupted) {
  // if (interrupted) {
  //   // TODO: Remove this once we have a limit switch and can end gracefully.
  //   m_intake->CloseCone();
  m_intake->CloseCone();
  m_intake->HoldGamePiece();

  // } else {
  //   m_intake->CloseCone();
  //   m_intake->HoldGamePiece();
  // }
}

// Returns true when the command should end.
bool IntakeConeCommand::IsFinished() {
#ifdef INTAKE_SWITCH_ENABLED
  return m_intake->IsGamePieceSwitchPressed();
#else
  return false;
#endif
}
