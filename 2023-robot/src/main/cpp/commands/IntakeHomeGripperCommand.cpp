// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/IntakeHomeGripperCommand.h"

IntakeHomeGripperCommand::IntakeHomeGripperCommand(IntakeSubsystem* intake) : m_intake(intake) {
  SetName("IntakeHomeGripperCommand");
}

// Called when the command is initially scheduled.
void IntakeHomeGripperCommand::Initialize() {
  m_intake->SetGripperDutyCycle(-0.2);
}

// Called repeatedly when this Command is scheduled to run
void IntakeHomeGripperCommand::Execute() {}

// Called once the command ends or is interrupted.
void IntakeHomeGripperCommand::End(bool interrupted) {
  if (interrupted || m_timedOut) {
    m_intake->SetGripperOff();
  } else {
    m_intake->ZeroGripper();
  }
}

// Returns true when the command should end.
bool IntakeHomeGripperCommand::IsFinished() {
  if (fabs(m_intake->GetGripperCurrent()) > IntakeConstants::kGripperHomeCurrent) {
    return true;
  }
  return false;
}
