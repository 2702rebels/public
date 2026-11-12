#include "commands/auto/Auto_ScoreAndBalance.h"

#include <frc2/command/ParallelCommandGroup.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/ScheduleCommand.h>
#include <frc2/command/WaitCommand.h>

#include "commands/ArmMoveCommand.h"
#include "commands/AutoBalanceCommand.h"
#include "commands/DriveTrajectoryCommand.h"
#include "commands/IntakeCubeAutoCommand.h"
#include "commands/ScoreConeCommand.h"
#include "commands/ScoreCubeCommand.h"
#include "frc2/command/InstantCommand.h"

constexpr auto origin = frc::Translation2d { 0_m, 0_m };

Auto_ScoreAndBalance::Auto_ScoreAndBalance(
  Alliance alliance, DriveSubsystem* drive, ArmSubsystem* arm, IntakeSubsystem* intake) {
  AddCommands(
    frc2::InstantCommand([intake] { intake->CloseCube(); }, {}),
    ArmMoveCommand(arm, ArmPosition::SCORE_HIGH_CUBE),
    frc2::ParallelCommandGroup(
      frc2::SequentialCommandGroup(
        ScoreCubeCommand(arm, intake),
        ArmMoveCommand(arm, ArmPosition::INTAKE_FLOOR_CUBE),
        frc2::ParallelRaceGroup(IntakeCubeAutoCommand(intake), frc2::WaitCommand(2.5_s))),
      DriveTrajectoryCommand(
        {
          Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(180_deg)),
          Waypoint(frc::Pose2d { 1_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 2_m, 0_m, 0_deg }),
          Waypoint(frc::Pose2d { 3.5_m, 0_m, 0_deg }, 1_mps),
          Waypoint(frc::Pose2d { 4.8_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
        },
        origin,
        true,  // reset odometry
        true,  // stop based on timer
        drive)),
    frc2::ParallelCommandGroup(
      ArmMoveCommand(arm, ArmPosition::SCORE_MID_CUBE),
      DriveTrajectoryCommand(
        {
          Waypoint(frc::Pose2d { 4.8_m, 0_m, 180_deg }, frc::Rotation2d(0_deg)),
          Waypoint(frc::Pose2d { 3.6_m, 0_m, 180_deg }),
          Waypoint(frc::Pose2d { 2.6_m, 0_m, 180_deg }),
          Waypoint(frc::Pose2d { 1.3_m, 0_m, 180_deg }, 1_mps),
          Waypoint(frc::Pose2d { 0.2_m, 0_m, 180_deg }, frc::Rotation2d(181_deg)),
        },
        origin,
        false,
        true,  // stop based on timer
        drive)),
    frc2::ParallelCommandGroup(
      ScoreCubeCommand(arm, intake),
      DriveTrajectoryCommand(
        alliance == Alliance::RED ?
        std::vector<Waypoint>
        {
          Waypoint(frc::Pose2d { 0.2_m, 0_m, 90_deg }, frc::Rotation2d(181_deg)),
          Waypoint(frc::Pose2d { 0.2_m, 1_m, 90_deg }),
          Waypoint(frc::Pose2d { 0.5_m, 2.5_m, 0_deg }, frc::Rotation2d(90_deg), 1_mps),
        } :
        std::vector<Waypoint>
        {
          Waypoint(frc::Pose2d { 0.2_m, 0_m, -90_deg }, frc::Rotation2d(181_deg)),
          Waypoint(frc::Pose2d { 0.2_m, -1_m, -90_deg }),
          Waypoint(frc::Pose2d { 0.5_m, -2.5_m, 0_deg }, frc::Rotation2d(-90_deg), 1_mps),
        },
        origin,
        false,
        true,  // stop based on timer,
        drive)),
    AutoBalanceCommand(drive, Side::INNER));
}