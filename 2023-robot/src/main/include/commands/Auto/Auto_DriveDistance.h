#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"

class Auto_DriveDistance
  : public frc2::CommandHelper<frc2::SequentialCommandGroup, Auto_DriveDistance> {
 public:
  Auto_DriveDistance(DriveSubsystem* drive);
};
