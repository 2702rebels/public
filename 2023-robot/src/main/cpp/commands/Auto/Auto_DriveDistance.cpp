#include "commands/auto/Auto_DriveDistance.h"

#include "commands/DriveTrajectoryCommand.h"

Auto_DriveDistance::Auto_DriveDistance(DriveSubsystem* drive) {
  AddCommands(DriveTrajectoryCommand(
    {
      Waypoint(frc::Pose2d { 0_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
      Waypoint(frc::Pose2d { 1_m, 0_m, 0_deg }, frc::Rotation2d(0_deg)),
    },
    frc::Translation2d { 0_m, 0_m },
    true,
    true,
    drive));
}
