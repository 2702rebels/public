// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ScoreCubeCommand.h"

#include <frc/DataLogManager.h>
#include <frc/Timer.h>

ScoreCubeCommand::ScoreCubeCommand(ArmSubsystem* arm, IntakeSubsystem* intake) :
  m_arm(arm),
  m_intake(intake) {
  SetName("ScoreCubeCommand");
  // Use addRequirements() here to declare subsystem dependencies.
  AddRequirements(arm);
  AddRequirements(intake);
}

// Called when the command is initially scheduled.
void ScoreCubeCommand::Initialize() {
  frc::DataLogManager::Log("ScoreCubeCommand::Initialize");

  m_startTime = frc::Timer::GetFPGATimestamp();
  m_intake->OuttakeCube();
  m_arm->Move(0.0_in);
}

// Called repeatedly when this Command is scheduled to run
void ScoreCubeCommand::Execute() {}

// Called once the command ends or is interrupted.
void ScoreCubeCommand::End(bool interrupted) {
  m_intake->Stop();
  m_intake->CloseCube();
  m_arm->Move(0.0_deg, 0.0_in);
}

// Returns true when the command should end.
bool ScoreCubeCommand::IsFinished() {
  if (
    m_arm->IsAtCommandedPosition() &&
    frc::Timer::GetFPGATimestamp() > m_startTime + IntakeConstants::kOuttakingTime) {
    return true;
  }
  return false;
}
