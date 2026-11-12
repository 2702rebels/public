#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "subsystems/ArmSubsystem.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/IntakeSubsystem.h"

class Auto_ScoreAndOverAndBalance
  : public frc2::CommandHelper<frc2::SequentialCommandGroup, Auto_ScoreAndOverAndBalance> {
 public:
  Auto_ScoreAndOverAndBalance(DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake);
};
