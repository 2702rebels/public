#pragma once

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

class Auto_DoNothing : public frc2::CommandHelper<frc2::CommandBase, Auto_DoNothing> {
 public:
  Auto_DoNothing();
};
