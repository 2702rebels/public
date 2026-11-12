// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ArmMoveCommand.h"

#include "frc/DataLogManager.h"

ArmMoveCommand::ArmMoveCommand(ArmSubsystem* arm, units::degree_t angle, units::inch_t distance) :
  m_arm(arm),
  m_axis1Angle(angle),
  m_axis2Distance(distance) {
  SetName("ArmMoveCommand");
  AddRequirements(arm);
}

ArmMoveCommand::ArmMoveCommand(ArmSubsystem* arm, ArmPosition position) :
  m_arm(arm),
  m_position(position) {
  SetName("ArmMoveCommand(Position)");
  AddRequirements(arm);
  m_isUsePosition = true;
}

ArmMoveCommand::ArmMoveCommand(ArmSubsystem* arm, units::degree_t angle) :
  m_arm(arm),
  m_axis1Angle(angle) {
  SetName("ArmMoveCommand(Angle)");
  AddRequirements(arm);
  m_isAngleOnly = true;
}

ArmMoveCommand::ArmMoveCommand(ArmSubsystem* arm, units::inch_t distance) :
  m_arm(arm),
  m_axis2Distance(distance) {
  SetName("ArmMoveCommand(Distance)");
  AddRequirements(arm);
  m_isDistanceOnly = true;
}

// Called when the command is initially scheduled.
void ArmMoveCommand::Initialize() {
  frc::DataLogManager::Log("ArmMoveCommand::Initialize");

  if (m_isUsePosition) {
    m_arm->Move(m_position);
  } else if (m_isAngleOnly) {
    m_arm->Move(m_axis1Angle);
  } else if (m_isDistanceOnly) {
    m_arm->Move(m_axis2Distance);
  } else {
    m_arm->Move(m_axis1Angle, m_axis2Distance);
  }
}

// Called repeatedly when this Command is scheduled to run
void ArmMoveCommand::Execute() {}

// Called once the command ends or is interrupted.
void ArmMoveCommand::End(bool interrupted) {
  frc::DataLogManager::Log(fmt::format("ArmMoveCommand::End {}", interrupted));
}

// Returns true when the command should end.
bool ArmMoveCommand::IsFinished() {
  if (m_arm->IsAtCommandedPosition()) {
    return true;
  }
  return false;
}
