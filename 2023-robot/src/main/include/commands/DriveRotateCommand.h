#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Rotation2d.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>

#include "subsystems/DriveSubsystem.h"
#include "utils/Waypoint.h"
#include "utils/WaypointTrajectory.h"

class DriveRotateCommand : public frc2::CommandHelper<frc2::CommandBase, DriveRotateCommand> {
 public:
  DriveRotateCommand(std::function<frc::Rotation2d()> desiredRotation, DriveSubsystem* drive);

  DriveRotateCommand(const frc::Rotation2d& desiredRotation, DriveSubsystem* drive) :
    DriveRotateCommand([desiredRotation] { return desiredRotation; }, drive) {}

  DriveRotateCommand(units::degree_t desiredAngle, DriveSubsystem* drive) :
    DriveRotateCommand(frc::Rotation2d(desiredAngle), drive) {}

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  /**
   * Determines whether robot is at goal and the command is running.
   */
  bool AtGoal() const;

  /**
   * Determines whether robot is at goal within specified tolerances.
   */
  bool AtGoal(units::radian_t rotationTolerance) const;

  /**
   * Determines whether command is running.
   */
  bool IsRunning() const {
    return m_running;
  }

 private:
  bool m_running = false;
  DriveSubsystem* m_drive;
  std::function<frc::Rotation2d()> m_desiredRotation;
  frc::ProfiledPIDController<units::radians> m_thetaController;
};
