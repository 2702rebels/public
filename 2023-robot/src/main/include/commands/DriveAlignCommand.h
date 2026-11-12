#pragma once

#include <frc/controller/ProfiledPIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc2/command/CommandHelper.h>
#include <units/angle.h>
#include <units/length.h>

#include <optional>

#include "subsystems/DriveSubsystem.h"

class DriveAlignCommand : public frc2::CommandHelper<frc2::CommandBase, DriveAlignCommand> {
 public:
  /**
   * Drives robot to the target position using the beam vector to a vision target.
   * The target position must be specified relative to the april tag in the reference
   * frame of the beam vector, i.e. negative Y would be towards the robot, negative X
   * would be to the left of the robot.
   *
   * This command first tries to align the robot laterally using X position from
   * the beam vector and then proceeds driving forward once X is within tolerance.
   *
   * This command assumes orientation and position of the vision target to be placed
   * at the wall opposite to the "alliance wall".
   *
   * @param beam Beam vector (e.g. to the april tag)
   * @param target Target position
   */
  DriveAlignCommand(
    std::function<std::optional<frc::Pose2d>()> beam,
    std::function<frc::Translation2d()> target,
    DriveSubsystem* drive);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;

  /**
   * Determines whether we have achieved alignment.
   */
  bool IsAligned() const {
    return m_aligned;
  }

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
  DriveSubsystem* m_drive;
  std::function<std::optional<frc::Pose2d>()> m_beam;
  std::function<frc::Translation2d()> m_target;
  frc::ProfiledPIDController<units::meter> m_alignController;
  frc::ProfiledPIDController<units::meter> m_driveController;
  frc::ProfiledPIDController<units::radians> m_thetaController;
  units::degree_t m_tolerance;
  units::meter_t m_offset;
  frc::Pose2d m_desiredPose;
  bool m_aligned;

  static constexpr frc::Translation2d Zero = {};
};
