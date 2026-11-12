package frc.robot.subsystems.intake;

import org.littletonrobotics.junction.AutoLog;

public interface IntakeIO {
  @AutoLog
  public static class IntakeIOInputs {
    public double velocityRPM = 0.0;
    public double temperature = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public boolean beamBreak = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(IntakeIOInputs inputs) {}

  /** Sets the motor speed. */
  public default void setSpeed(double speed) {}

  /** Stops the motor. */
  public default void stop() {}
}
