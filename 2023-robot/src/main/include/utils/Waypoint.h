#pragma once

#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <units/velocity.h>

#include <optional>

class Waypoint {
 public:
  /**
   * Creates a waypoint from the pose, holonomic rotation and speed.
   */
  Waypoint(
    const frc::Pose2d& pose,
    const frc::Rotation2d& rotation,
    const units::meters_per_second_t speed) :
    position(pose.Translation()),
    direction(pose.Rotation()),
    rotation(rotation),
    speed(speed) {}

  /**
   * Creates a waypoint from the pose and holonomic rotation.
   */
  Waypoint(const frc::Pose2d& pose, const frc::Rotation2d& rotation) :
    position(pose.Translation()),
    direction(pose.Rotation()),
    rotation(rotation),
    speed(std::nullopt) {}

  /**
   * Creates a waypoint from the pose and speed.
   */
  Waypoint(const frc::Pose2d& pose, const units::meters_per_second_t speed) :
    position(pose.Translation()),
    direction(pose.Rotation()),
    speed(speed) {}

  /**
   * Creates a waypoint from the pose without holonomic rotation.
   */
  Waypoint(const frc::Pose2d& pose) :
    position(pose.Translation()),
    direction(pose.Rotation()),
    rotation(std::nullopt),
    speed(std::nullopt) {}

  /**
   * Creates a waypoint from the translation and direction without holonomic rotation.
   */
  Waypoint(const frc::Translation2d& position, const frc::Rotation2d& rotation) :
    position(position),
    direction(rotation),
    rotation(std::nullopt),
    speed(std::nullopt) {}

  /**
   * Creates a waypoint from the position without heading or holonomic rotation.
   */
  Waypoint(const frc::Translation2d& position) :
    position(position),
    direction(std::nullopt),
    rotation(std::nullopt),
    speed(std::nullopt) {}

  /**
   * Creates a waypoint from the position without heading or holonomic rotation and speed.
   */
  Waypoint(const frc::Translation2d& position, const units::meters_per_second_t speed) :
    position(position),
    direction(std::nullopt),
    rotation(std::nullopt),
    speed(speed) {}

  /**
   * Translation of the robot.
   */
  const frc::Translation2d position;

  /**
   * Directional heading of the robot.
   */
  const std::optional<const frc::Rotation2d> direction;

  /**
   * Holonomic rotation.
   */
  const std::optional<const frc::Rotation2d> rotation;

  /**
   * Speed.
   */
  const std::optional<units::meters_per_second_t> speed;
};
