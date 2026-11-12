#pragma once

#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"
#include "utils/CustomHolonomicDriveController.h"
#include "utils/Waypoint.h"
#include "utils/WaypointTrajectory.h"

class DriveTrajectoryCommand
  : public frc2::CommandHelper<frc2::CommandBase, DriveTrajectoryCommand> {
 public:
  DriveTrajectoryCommand(
    const std::vector<Waypoint>& waypoints,
    const frc::Translation2d& origin,
    bool resetOdometry,
    bool stopByTimer,
    bool printTrajectory,
    DriveSubsystem* drive);

  DriveTrajectoryCommand(
    const std::vector<Waypoint>& waypoints,
    const frc::Translation2d& origin,
    bool resetOdometry,
    bool stopByTimer,
    DriveSubsystem* drive) :
    DriveTrajectoryCommand(waypoints, origin, resetOdometry, stopByTimer, false, drive) {}

  DriveTrajectoryCommand(
    const std::vector<Waypoint>& waypoints,
    const frc::Translation2d& origin,
    bool resetOdometry,
    DriveSubsystem* drive) :
    DriveTrajectoryCommand(waypoints, origin, resetOdometry, false, false, drive) {}

  DriveTrajectoryCommand(
    const std::vector<Waypoint>& waypoints,
    const frc::Translation2d& origin,
    DriveSubsystem* drive) :
    DriveTrajectoryCommand(waypoints, origin, false, false, false, drive) {}

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  /**
   * Determines whether robot is at goal and the command is running.
   */
  bool AtGoal() const;

  /**
   * Determines whether command is running.
   */
  bool IsRunning() const {
    return m_running;
  }

 private:
  bool m_running = false;
  DriveSubsystem* m_drive;
  frc::Timer m_timer;
  WaypointTrajectory m_trajectory;
  CustomHolonomicDriveController m_controller;
  frc::Pose2d m_finalPose;
  const bool m_resetOdometry;
  const bool m_stopByTimer;
};
