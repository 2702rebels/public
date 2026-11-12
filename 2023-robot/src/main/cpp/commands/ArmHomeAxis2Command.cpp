// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmHomeAxis2Command.h"

#include <frc/DataLogManager.h>
#include <frc/Timer.h>

ArmHomeAxis2Command::ArmHomeAxis2Command(ArmSubsystem* arm) : m_arm(arm) {
  SetName("ArmHomeAxis2Command");
  AddRequirements(arm);
  // Use addRequirements() here to declare subsystem dependencies.
}

// Called when the command is initially scheduled.
void ArmHomeAxis2Command::Initialize() {
  m_isHomed = false;
  m_isTimedOut = false;
  m_homingStartTime = frc::Timer::GetFPGATimestamp();
  m_cyclesAboveCurrentLimit = 0;
  m_arm->SetAxis2DutyCycle(-0.6);
  m_arm->DisableControlAxis2();
}

// Called repeatedly when this Command is scheduled to run
void ArmHomeAxis2Command::Execute() {
  if (m_arm->GetAxis2Current() >= ArmConstants::kAxis2HomeCurrentThreshold) {
    if (m_cyclesAboveCurrentLimit++ > 5) {
      m_arm->ZeroAxis2();
      m_isHomed = true;
    }
  } else {
    m_cyclesAboveCurrentLimit = 0;
  }

  if (
    m_homingStartTime + units::second_t(ArmConstants::kAxis2HomeTimeoutInSeconds) <=
    frc::Timer::GetFPGATimestamp()) {
    m_isTimedOut = true;
  }
}

// Called once the command ends or is interrupted.
void ArmHomeAxis2Command::End(bool interrupted) {
  if (interrupted || m_isTimedOut) {
    m_arm->SetAxis2Off();

    if (m_isTimedOut) {
      frc::DataLogManager::Log(fmt::format("HomeAxis2: Timed out"));
    }
  }
}

// Returns true when the command should end.
bool ArmHomeAxis2Command::IsFinished() {
  if (m_isHomed || m_isTimedOut) {
    return true;
  }
  return false;
}
