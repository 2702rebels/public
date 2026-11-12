// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/SetLedCommand.h"

#include <frc/DataLogManager.h>
#include <frc/Timer.h>

SetLedCommand::SetLedCommand(
  LedSubsystem* subsystem,
  int r,
  int g,
  int b,
  LEDPattern pattern,
  units::millisecond_t period,
  units::millisecond_t timeToDisplay) :
  m_ledSubsystem { subsystem },
  m_r { r },
  m_g { g },
  m_b { b },
  m_pattern { pattern },
  m_period { period },
  m_timeToDisplay { timeToDisplay } {
  SetName("SetLedCommand");
  AddRequirements(m_ledSubsystem);
}

// Called when the command is initially scheduled.
void SetLedCommand::Initialize() {
  frc::DataLogManager::Log("Init LED Command");
  if (m_timeToDisplay > 0_ms) {
    m_ledSubsystem->StorePattern();
    m_startTime = frc::Timer::GetFPGATimestamp();
  }

  m_ledSubsystem->SetRGB(m_r, m_g, m_b);
  m_ledSubsystem->SetPattern(m_pattern, m_period);
}

// Called repeatedly when this Command is scheduled to run
void SetLedCommand::Execute() {}

// Called once the command ends or is interrupted.
void SetLedCommand::End(bool interrupted) {
  frc::DataLogManager::Log("End LED Command");
  if (m_timeToDisplay > 0_ms) {
    m_ledSubsystem->RestorePattern();
  }
}

// Returns true when the command should end.
bool SetLedCommand::IsFinished() {
  if (m_timeToDisplay <= 0_ms || frc::Timer::GetFPGATimestamp() > m_startTime + m_timeToDisplay) {
    return true;
  }
  return false;
}
