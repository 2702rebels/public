#include "commands/auto/Auto_ScoreAndOverAndBalance.h"

#include "Field.h"
#include "commands/ArmMoveCommand.h"
#include "commands/AutoBalanceCommand.h"
#include "commands/DriveOverBridgeCommand.h"
#include "commands/DriveRotateCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "commands/ScoreCubeCommand.h"
#include "frc2/command/InstantCommand.h"
#include "frc2/command/ParallelRaceGroup.h"
#include "frc2/command/WaitCommand.h"

Auto_ScoreAndOverAndBalance::Auto_ScoreAndOverAndBalance(
  DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
  AddCommands(
    frc2::InstantCommand([intake] { intake->CloseCube(); }, {}),
    ArmMoveCommand(arm, ArmPosition::SCORE_HIGH_CUBE),
    ScoreCubeCommand(arm, intake),
    frc2::InstantCommand([drive] { drive->ResetOdometry(Field::GetFieldPose(Field::CUBE_2)); }, {}),
    DriveRotateCommand(90_deg, drive),
    frc2::ParallelRaceGroup(DriveOverBridgeCommand(drive), frc2::WaitCommand(6_s)),
    AutoBalanceCommand(drive, Side::OUTER));
}
