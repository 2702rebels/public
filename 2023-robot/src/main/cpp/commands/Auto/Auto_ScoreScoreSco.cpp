
#include "commands/Auto/Auto_ScoreScoreSco.h"

#include <frc2/command/InstantCommand.h>
#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/WaitCommand.h>

#include "commands/ArmMoveCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "commands/IntakeConeCommand.h"
#include "commands/IntakeCubeCommand.h"
#include "commands/ScoreConeCommand.h"
#include "commands/ScoreCubeCommand.h"

Auto_ScoreScoreSco::Auto_ScoreScoreSco(
  DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
  AddCommands(
    frc2::InstantCommand([intake] { intake->CloseCone(); }, {}),
    ArmMoveCommand(arm, 78_deg, 0_in),
    ArmMoveCommand(arm, ArmPosition::SCORE_HIGH_CONE),
    frc2::ParallelCommandGroup(
      frc2::SequentialCommandGroup(
        ScoreConeCommand(arm, intake),
        ArmMoveCommand(arm, ArmPosition::INTAKE_FLOOR_CUBE),
        frc2::ParallelRaceGroup(IntakeCubeCommand(intake), frc2::WaitCommand(2_s))),
      DriveTrajectoryCommand(
        {
          Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(180_deg)),
          Waypoint(frc::Pose2d { 1_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 2_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 3_m, 0_m, 0_deg }, 1_mps),
          Waypoint(frc::Pose2d { 4.6_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
        },
        frc::Translation2d { 0_m, 0_m },
        true,
        false,
        drive)),
    frc2::ParallelCommandGroup(
      ArmMoveCommand(arm, ArmPosition::SCORE_HIGH_CUBE),
      DriveTrajectoryCommand(
        {
          Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(180_deg)),
          Waypoint(frc::Pose2d { 1_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 2_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 3_m, 0_m, 0_deg }, 1_mps),
          Waypoint(frc::Pose2d { 4.6_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
        },
        frc::Translation2d { 0_m, 0_m },
        true,
        false,
        drive)),
    frc2::ParallelCommandGroup(
      frc2::SequentialCommandGroup(
        ScoreCubeCommand(arm, intake),
        ArmMoveCommand(arm, ArmPosition::INTAKE_FLOOR_CONE),
        frc2::ParallelRaceGroup(IntakeConeCommand(intake), frc2::WaitCommand(2_s))),
      DriveTrajectoryCommand(
        {
          Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(180_deg)),
          Waypoint(frc::Pose2d { 1_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 2_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 3_m, 0_m, 0_deg }, 1_mps),
          Waypoint(frc::Pose2d { 4.6_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
        },
        frc::Translation2d { 0_m, 0_m },
        true,
        false,
        drive)),
    frc2::ParallelCommandGroup(
      ArmMoveCommand(arm, ArmPosition::STOWED),
      DriveTrajectoryCommand(
        {
          Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(180_deg)),
          Waypoint(frc::Pose2d { 1_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 2_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 3_m, 0_m, 0_deg }, 1_mps),
          Waypoint(frc::Pose2d { 4.6_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
        },
        frc::Translation2d { 0_m, 0_m },
        true,
        false,
        drive)));
}
