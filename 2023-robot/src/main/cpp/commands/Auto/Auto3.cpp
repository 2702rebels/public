#include "commands/auto/Auto3.h"

#include "Field.h"
#include "commands/AutoBalanceCommand.h"
#include "commands/DriveOverBridgeCommand.h"
#include "commands/DriveRotateCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "frc2/command/InstantCommand.h"
#include "frc2/command/ParallelRaceGroup.h"
#include "frc2/command/WaitCommand.h"

Auto3::Auto3(DriveSubsystem* drive) {
  AddCommands(
    frc2::InstantCommand([drive] { drive->ResetOdometry(Field::GetFieldPose(Field::CUBE_2)); }, {}),
    DriveRotateCommand(90_deg, drive),
    frc2::ParallelRaceGroup(DriveOverBridgeCommand(drive), frc2::WaitCommand(6_s)),
    AutoBalanceCommand(drive, Side::OUTER));
}
