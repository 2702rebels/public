#pragma once

#include <frc2/command/CommandHelper.h>
#include <frc2/command/SequentialCommandGroup.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/LedSubsystem.h"

class Auto2 : public frc2::CommandHelper<frc2::SequentialCommandGroup, Auto2> {
 public:
  Auto2(DriveSubsystem* drive, LedSubsystem* led);
};
