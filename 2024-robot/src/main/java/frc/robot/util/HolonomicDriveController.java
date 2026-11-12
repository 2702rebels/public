package frc.robot.util;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class HolonomicDriveController {
  private boolean enabled = true;
  private Pose2d tolerance;
  private final PIDController xController;
  private final PIDController yController;
  private final PIDController thetaController;

  public HolonomicDriveController(
      PIDController xController, PIDController yController, PIDController thetaController) {
    this.xController = xController;
    this.yController = yController;
    this.thetaController = thetaController;
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);
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
  public ChassisSpeeds calculate(
      Pose2d currentPose,
      Pose2d trajectoryPose,
      double desiredLinearVelocityMetersPerSecond,
      Rotation2d desiredRotation,
      double desiredAngularVelocityRadiansPerSecond) {
    // calculate feed-forward velocities (field-relative)
    final var xFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getCos();
    final var yFF = desiredLinearVelocityMetersPerSecond * trajectoryPose.getRotation().getSin();
    final var thetaFF = desiredAngularVelocityRadiansPerSecond;

    if (!enabled) {
      return ChassisSpeeds.fromFieldRelativeSpeeds(xFF, yFF, thetaFF, currentPose.getRotation());
    }

    // calculate feedback velocities (based on position error)
    final var xFeedback = xController.calculate(currentPose.getX(), trajectoryPose.getX());
    final var yFeedback = yController.calculate(currentPose.getY(), trajectoryPose.getY());
    final var thetaFeedback =
        thetaController.calculate(
            currentPose.getRotation().getRadians(), desiredRotation.getRadians());

    return ChassisSpeeds.fromFieldRelativeSpeeds(
        xFF + xFeedback, yFF + yFeedback, thetaFF + thetaFeedback, currentPose.getRotation());
  }

  /**
   * Sets the pose error which is considered tolerable for use with atGoal.
   *
   * @param tolerance Pose error which is tolerable.
   */
  public void setTolerance(Pose2d tolerance) {
    this.tolerance = tolerance;

    final var translation = tolerance.getTranslation();
    xController.setTolerance(translation.getX());
    yController.setTolerance(translation.getY());
    thetaController.setTolerance(tolerance.getRotation().getRadians());
  }

  /** Gets the pose tolerance. */
  public Pose2d getTolerance() {
    return tolerance;
  }

  /** Returns the rotation PIDController. */
  public PIDController getThetaController() {
    return thetaController;
  }

  /** Returns the X PIDController. */
  public PIDController getXController() {
    return xController;
  }

  /** Returns the Y PIDController. */
  public PIDController getYController() {
    return yController;
  }

  /** Returns true if the pose error is within tolerance of the goal. */
  public boolean atGoal() {
    return xController.atSetpoint() && yController.atSetpoint() && thetaController.atSetpoint();
  }

  /**
   * Enables and disables the controller for troubleshooting purposes. When calculate() is called on
   * a disabled controller, only feed-forward values are returned.
   *
   * @param enabled If the controller is enabled or not.
   */
  public void setEnabled(boolean enabled) {
    this.enabled = enabled;
  }

  /** Resets the controller. */
  public void reset() {
    thetaController.reset();
    xController.reset();
    yController.reset();
  }
}
