package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
  @AutoLog
  public static class PivotIOInputs {
    public double measuredPositionRadians = 0.0;
    public double measuredVelocityRadiansPerSecond = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(PivotIOInputs inputs) {}

  /** Enables or Disables brake mode on the joint. */
  public default void setBrakeMode(boolean enable) {}

  /** Confiure the PID for the joint. */
  public default void configureJointPID(double kP, double kI, double kD) {}

  /** Sets the target angle for the joint PID. */
  public default void setJointAngle(double angleRadians) {}

  /** Sets the target angle for the joint PID with a FF term. */
  public default void setJointAngle(double angleRadians, double ff) {}

  /** Stops the motor. */
  public default void stop() {}
}
