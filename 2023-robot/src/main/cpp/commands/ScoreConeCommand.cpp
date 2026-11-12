// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ScoreConeCommand.h"

#include <frc/DataLogManager.h>

ScoreConeCommand::ScoreConeCommand(ArmSubsystem* arm, IntakeSubsystem* intake) :
  m_arm(arm),
  m_intake(intake) {
  SetName("ScoreConeCommand");
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm);
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void ScoreConeCommand::Initialize() {
  m_armisClear = false;
  m_intake->PlaceCone();
  m_arm->Move(0.0_in);
}

// Called repeatedly when this Command is scheduled to run
void ScoreConeCommand::Execute() {
  if (m_arm->IsAtCommandedPosition()) {
    m_armisClear = true;
  }
}

// Called once the command ends or is interrupted.
void ScoreConeCommand::End(bool interrupted) {
  m_intake->Stop();
  m_intake->CloseCone();
  m_arm->Move(0.0_deg, 0.0_in);
}

// Returns true when the command should end.
bool ScoreConeCommand::IsFinished() {
  if (m_armisClear) {
    return true;
  }
  return false;
}