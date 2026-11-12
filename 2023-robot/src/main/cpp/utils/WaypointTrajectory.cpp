#include "utils/WaypointTrajectory.h"

#include <frc/DataLogManager.h>
#include <frc/trajectory/TrajectoryGenerator.h>

#include <algorithm>

WaypointTrajectory::WaypointTrajectory(
  const std::vector<Waypoint>& waypoints,
  const frc::Translation2d& origin,
  frc::SwerveDriveKinematics<4U>& kinematics,
  units::meters_per_second_t maxVelocity,
  units::meters_per_second_squared_t maxAcceleration,
  units::radians_per_second_t maxAngularVelocity,
  units::radians_per_second_squared_t maxAngularAcceleration) {
  if (waypoints.size() < 2) {
    throw std::runtime_error("Trajectory requires at least 2 waypoints");
  }

  std::vector<frc::Pose2d> poses;
  std::vector<std::tuple<unsigned int, units::meters_per_second_t>> speedControlPoints;

  // if directional heading is not specified for a position, auto-generate one
  // as the vector towards the next position, for the last waypoint retain
  // current (previous) heading
  unsigned int index = 0;
  for (const auto& waypoint : waypoints) {
    const auto position = waypoint.position + origin;

    if (waypoint.direction.has_value()) {
      poses.push_back({ position, waypoint.direction.value() });
    } else if (index + 1 < waypoints.size()) {
      const auto direction = (waypoints.at(index + 1).position - waypoint.position).Angle();
      poses.push_back({ position, direction });
    } else {
      const auto direction = (waypoint.position - waypoints.at(index - 1).position).Angle();
      poses.push_back({ position, direction });
    }

    if (waypoint.speed.has_value()) {
      speedControlPoints.push_back({ index, waypoint.speed.value() });
    }

    ++index;
  }

  // if we have waypoints controlling speed we have to split the path into
  // individual trajectories and splice them together at the speed control
  // points by setting end and start velocities to the desired speed
  if (speedControlPoints.size() > 0) {
    unsigned int startIndex = 0;
    units::meters_per_second_t startVelocity = 0_mps;
    m_trajectory = frc::Trajectory();

    for (const auto& controlPoint : speedControlPoints) {
      const auto endIndex = std::get<0>(controlPoint);
      const auto endVelocity = std::get<1>(controlPoint);

      // deal with the situation when first waypoint is a control point
      if (endIndex > startIndex) {
        std::vector<frc::Pose2d> leg(poses.begin() + startIndex, poses.begin() + endIndex + 1);

        const auto trajectoryVelocity = units::math::max(startVelocity, endVelocity);
        frc::TrajectoryConfig config(
          trajectoryVelocity > 0_mps ? trajectoryVelocity : maxVelocity, maxAcceleration);

        config.SetKinematics(kinematics);
        config.SetEndVelocity(endVelocity);
        config.SetStartVelocity(startVelocity);

        // splice trajectories
        m_trajectory =
          m_trajectory + FixTrajectory(frc::TrajectoryGenerator::GenerateTrajectory(leg, config));
      }

      startVelocity = endVelocity;
      startIndex = endIndex;
    }

    // last leg
    if (poses.begin() + startIndex < poses.end() - 1) {
      std::vector<frc::Pose2d> leg(poses.begin() + startIndex, poses.end());

      const auto trajectoryVelocity = units::math::max(startVelocity, m_endVelocity);
      frc::TrajectoryConfig config(
        trajectoryVelocity > 0_mps ? trajectoryVelocity : maxVelocity, maxAcceleration);

      m_endVelocity = 0_mps;  // stop at the end
      config.SetKinematics(kinematics);
      config.SetEndVelocity(m_endVelocity);
      config.SetStartVelocity(startVelocity);

      // splice trajectories
      m_trajectory =
        m_trajectory + FixTrajectory(frc::TrajectoryGenerator::GenerateTrajectory(leg, config));
    } else {
      m_endVelocity = startVelocity;
    }
  } else {
    frc::TrajectoryConfig config(maxVelocity, maxAcceleration);
    config.SetKinematics(kinematics);

    m_trajectory = FixTrajectory(frc::TrajectoryGenerator::GenerateTrajectory(poses, config));
  }

  // now that we have constructed a drive trajectory we calculate a profile for
  // holonomic rotations based on the total time of the drive path and subject
  // to maximum velocity and acceleration constraints specified in the config
  const auto r0 = waypoints.front().rotation;
  const auto r1 = waypoints.back().rotation;

  m_rotationProfile = RotationProfile(
    r0.has_value() ? r0.value() : frc::Rotation2d(),
    r1.has_value() ? r1.value() : frc::Rotation2d(),
    m_trajectory.TotalTime(),
    maxAngularVelocity,
    maxAngularAcceleration);
}

frc::Trajectory WaypointTrajectory::FixTrajectory(const frc::Trajectory& trajectory) const {
  const auto states = trajectory.States();
  if (states.empty()) {
    return trajectory;
  }

  const auto endState = states.back();
  if (endState.acceleration.value() == 0) {
    return trajectory;
  }

  // create new trajectory with new end state
  std::vector<frc::Trajectory::State> newStates(states.begin(), states.end() - 1);
  newStates.emplace_back(frc::Trajectory::State {
    endState.t,
    endState.velocity,
    0_mps_sq,
    endState.pose,
    endState.curvature,
  });

  return frc::Trajectory(newStates);
}
