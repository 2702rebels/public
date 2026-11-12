#include "commands/auto/Auto_ScoreAndOut.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/WaitCommand.h>

#include "Field.h"
#include "commands/ArmMoveCommand.h"
#include "commands/AutoBalanceCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "commands/IntakeCubeAutoCommand.h"
#include "commands/ScoreConeCommand.h"
#include "frc2/command/InstantCommand.h"

Auto_ScoreAndOut::Auto_ScoreAndOut(
  Alliance alliance, DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
  AddCommands(
    frc2::InstantCommand([drive] { drive->ResetOdometry(Field::GetFieldPose(Field::CONE_1)); }, {}),
    frc2::InstantCommand([intake] { intake->CloseCone(); }, {}),
    ArmMoveCommand(arm, 68_deg, 0_in),
    ArmMoveCommand(arm, 70_deg, 17_in),
    ScoreConeCommand(arm, intake),
    DriveTrajectoryCommand(
      {
        Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(180_deg)),
        Waypoint(frc::Pose2d { 2.7_m, 0_m, 0_deg }, frc::Rotation2d(180_deg), 0.7_mps),
      },
      Field::GetFieldPose(Field::CONE_1).Translation(),
      true,  // reset odometry
      true,  // stop based on timer
      drive),
    frc2::ParallelCommandGroup(
      ArmMoveCommand(arm, ArmPosition::INTAKE_FLOOR_CUBE),
      frc2::ParallelRaceGroup(IntakeCubeAutoCommand(intake), frc2::WaitCommand(4_s)),
      DriveTrajectoryCommand(
        {
          Waypoint(frc::Pose2d { 2.7_m, 0_m, 0_deg }, frc::Rotation2d(180_deg), 0.7_mps),
          Waypoint(
            frc::Pose2d { 4.9_m, alliance == Alliance::RED ? -0.3_m : 0.3_m, 0_deg },
            frc::Rotation2d(0_deg)),
        },
        Field::GetFieldPose(Field::CONE_1).Translation(),
        false,
        true,  // stop based on timer
        drive)),
    frc2::InstantCommand([intake] { intake->CloseCube(); }, {}),
    ArmMoveCommand(arm, ArmPosition::STOWED));
}
