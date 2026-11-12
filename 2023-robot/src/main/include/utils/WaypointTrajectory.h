#pragma once

#include <frc/kinematics/SwerveDriveKinematics.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrapezoidProfile.h>
#include <units/angle.h>

#include <vector>

#include "RotationProfile.h"
#include "Waypoint.h"

class WaypointTrajectory {
 public:
  WaypointTrajectory() = default;

  WaypointTrajectory(
    const std::vector<Waypoint>& waypoints,
    const frc::Translation2d& origin,
    frc::SwerveDriveKinematics<4U>& kinematics,
    units::meters_per_second_t maxVelocity,
    units::meters_per_second_squared_t maxAcceleration,
    units::radians_per_second_t maxAngularVelocity,
    units::radians_per_second_squared_t maxAngularAcceleration);

  /**
   * Returns trajectory representing positional waypoints and headings.
   */
  const frc::Trajectory& Trajectory() const {
    return m_trajectory;
  }

  /**
   * Returns rotation profile that samples a sequence of holonomic rotations.
   */
  const RotationProfile& Rotations() const {
    return m_rotationProfile;
  }

  /**
   * Returns the initial pose of the trajectory.
   * @return The initial pose of the trajectory.
   */
  frc::Pose2d InitialPose() const {
    return frc::Pose2d(
      m_trajectory.InitialPose().Translation(), m_rotationProfile.InitialRotation());
  }

  /**
   * Returns the final pose of the trajectory.
   * @return The initial pose of the trajectory.
   */
  frc::Pose2d FinalPose() const {
    const auto states = m_trajectory.States();
    return frc::Pose2d(
      states.empty() ? frc::Translation2d() : states.back().pose.Translation(),
      m_rotationProfile.FinalRotation());
  }

  /**
   * Gets the end velocity of the trajectory.
   */
  units::meters_per_second_t GetEndVelocity() const {
    return m_endVelocity;
  }

 private:
  frc::Trajectory m_trajectory;
  RotationProfile m_rotationProfile;
  units::meters_per_second_t m_endVelocity = 0_mps;

  /**
   * Constructs "fixed" trajectory since WPILIB trajectory generator
   * sets the acceleration of the last state to non-zero causing
   * spliced trajectory to incorrectly interpolate speeds in-between.
   */
  frc::Trajectory FixTrajectory(const frc::Trajectory& trajectory) const;
};
