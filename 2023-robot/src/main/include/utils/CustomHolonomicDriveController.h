#pragma once

#include <frc/controller/PIDController.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Rotation2d.h>
#include <frc/kinematics/ChassisSpeeds.h>
#include <frc/trajectory/Trajectory.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "utils/RotationProfile.h"

class CustomHolonomicDriveController {
 public:
  CustomHolonomicDriveController(
    frc2::PIDController xController,
    frc2::PIDController yController,
    frc2::PIDController thetaController);

  /**
   * Returns true if the pose error is within tolerance of the goal.
   */
  bool AtGoal() const;

  /**
   * Sets the pose error which is considered tolerable for use with
   * AtGoal().
   *
   * @param tolerance Pose error which is tolerable.
   */
  void SetTolerance(const frc::Pose2d& tolerance) {
    m_poseTolerance = tolerance;

    const auto translation = tolerance.Translation();
    m_xController.SetTolerance(translation.X().value());
    m_yController.SetTolerance(translation.Y().value());
    m_thetaController.SetTolerance(tolerance.Rotation().Radians().value());
  }

  /**
   * Gets the pose tolerance.
   */
  const frc::Pose2d& GetTolerance() const {
    return m_poseTolerance;
  }

  /**
   * Returns the next output of the holonomic drive controller.
   *
   * @param currentPose The current pose, as measured by odometry or pose estimator.
   * @param trajectoryPose The desired trajectory pose, as sampled for the current time step.
   * @param desiredLinearVelocity The desired linear velocity.
   * @param desiredRotation The desired rotation.
   * @param desiredAngularVelocity The desired angular velocity.
   * @return The next output of the holonomic drive controller.
   */
  frc::ChassisSpeeds Calculate(
    const frc::Pose2d& currentPose,
    const frc::Pose2d& trajectoryPose,
    const units::meters_per_second_t desiredLinearVelocity,
    const frc::Rotation2d& desiredRotation,
    const units::radians_per_second_t desiredAngularVelocity);

  frc::ChassisSpeeds Calculate(
    const frc::Pose2d& currentPose,
    const frc::Trajectory::State& desiredState,
    const RotationProfile::State& desiredRotation);

  /**
   * Enables and disables the controller for troubleshooting purposes. When
   * Calculate() is called on a disabled controller, only feed-forward values
   * are returned.
   *
   * @param enabled If the controller is enabled or not.
   */
  void SetEnabled(bool enabled) {
    m_enabled = enabled;
  }

  /**
   * Returns the rotation PIDController.
   */
  frc2::PIDController& getThetaController() {
    return m_thetaController;
  }

  /**
   * Returns the X PIDController.
   */
  frc2::PIDController& getXController() {
    return m_xController;
  }

  /**
   * Returns the Y PIDController.
   */
  frc2::PIDController& getYController() {
    return m_yController;
  }

  /**
   * Resets the controller.
   */
  void Reset() {
    m_thetaController.Reset();
    m_xController.Reset();
    m_yController.Reset();
  }

 private:
  frc::Pose2d m_poseTolerance;
  bool m_enabled = true;

  frc2::PIDController m_xController;
  frc2::PIDController m_yController;
  frc2::PIDController m_thetaController;
};
