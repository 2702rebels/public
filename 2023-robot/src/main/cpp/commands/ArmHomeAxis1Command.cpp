// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmHomeAxis1Command.h"

#include <frc/DataLogManager.h>
#include <frc/Timer.h>

ArmHomeAxis1Command::ArmHomeAxis1Command(ArmSubsystem* arm) : m_arm(arm) {
  SetName("ArmHomeAxis1Command");
  AddRequirements(arm);
}

// Called when the command is initially scheduled.
void ArmHomeAxis1Command::Initialize() {
  frc::DataLogManager::Log(fmt::format("HomeAxis1Command: Initialize"));
  m_isHomed = false;
  m_isTimedOut = false;
  m_isLimitHit = false;
  m_isWorkingHardProtection = false;
  m_homingStartTime = frc::Timer::GetFPGATimestamp();

  if (!m_arm->GetAxis1LimitSwitch()) {
    m_arm->SetAxis1DutyCycle(-0.2);

  } else {
    m_isLimitHit = true;
    m_arm->SetAxis1DutyCycle(0.2);
  }
}

// Called repeatedly when this Command is scheduled to run
void ArmHomeAxis1Command::Execute() {
  if (!m_isLimitHit && m_arm->GetAxis1LimitSwitch()) {
    frc::DataLogManager::Log(
      fmt::format("LimitSwitchState: {} Inside First If", m_arm->GetAxis1LimitSwitch()));
    m_isLimitHit = true;
    m_arm->SetAxis1DutyCycle(0.2);
  }

  if (m_isLimitHit && !m_arm->GetAxis1LimitSwitch()) {
    frc::DataLogManager::Log(
      fmt::format("LimitSwitchState: {} Inside Second If", m_arm->GetAxis1LimitSwitch()));

    m_arm->ZeroAxis1();
    m_isHomed = true;
  }

  if (m_arm->GetAxis1Current() >= ArmConstants::kAxis1HomeCurrentThreshold) {
    m_isWorkingHardProtection = true;
  }

  if (
    m_homingStartTime + units::second_t(ArmConstants::kAxis1HomeTimeoutInSeconds) <=
    frc::Timer::GetFPGATimestamp()) {
    m_isTimedOut = true;
  }
}

// Called once the command ends or is interrupted.
void ArmHomeAxis1Command::End(bool interrupted) {
  frc::DataLogManager::Log(fmt::format("HomeAxis1Command: End"));
  if (interrupted) {
    m_arm->SetAxis1Off();

    if (m_isTimedOut) {
      frc::DataLogManager::Log(fmt::format("HomeAxis1Command: Timed out"));
    }

    if (m_isWorkingHardProtection) {
      frc::DataLogManager::Log(fmt::format("HomeAxis1Command: Working too hard during homing. "));
    }
  }
}

// Returns true when the command should end.
bool ArmHomeAxis1Command::IsFinished() {
  if (m_isHomed || m_isTimedOut || m_isWorkingHardProtection) {
    return true;
  }
  return false;
}
