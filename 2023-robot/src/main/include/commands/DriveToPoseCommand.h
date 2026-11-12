#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/length.h>

#include "subsystems/DriveSubsystem.h"

class DriveToPoseCommand : public frc2::CommandHelper<frc2::CommandBase, DriveToPoseCommand> {
 public:
  DriveToPoseCommand(
    std::function<frc::Pose2d()> desiredPose, DriveSubsystem* drive, bool endAtGoal = false);

  DriveToPoseCommand(
    const frc::Pose2d& desiredPose, DriveSubsystem* drive, bool endAtGoal = false) :
    DriveToPoseCommand([desiredPose] { return desiredPose; }, drive, endAtGoal) {}

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

  /**
   * Determines whether PID controllers are at goal and the command is running.
   */
  bool AtGoal() const;

  /**
   * Determines whether robot is at goal within specified tolerances.
   */
  bool AtGoal(units::meter_t distanceTolerance, units::radian_t rotationTolerance) const;

  /**
   * Determines whether command is running.
   */
  bool IsRunning() const {
    return m_running;
  }

 private:
  bool m_running = false;
  bool m_endAtGoal;
  DriveSubsystem* m_drive;
  std::function<frc::Pose2d()> m_desiredPose;
  frc::ProfiledPIDController<units::meter> m_driveController;
  frc::ProfiledPIDController<units::radians> m_thetaController;
};
