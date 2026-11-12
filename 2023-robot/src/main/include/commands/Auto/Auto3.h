#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

class Auto3 : public frc2::CommandHelper<frc2::SequentialCommandGroup, Auto3> {
 public:
  Auto3(DriveSubsystem* drive);
};
