package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double flywheelVelocity = 0.0;
    public double flywheelPosition = 0.0;
    public double flywheelAppliedVolts = 0.0;
    public double flywheelCurrentAmps = 0.0;

    public double followerVelocity = 0.0;
    public double followerAppliedVolts = 0.0;
    public double followerCurrentAmps = 0.0;

    public double feedPosition = 0.0;
    public double feedVelocity = 0.0;
    public double feedAppliedVolts = 0.0;
    public double feedCurrentAmps = 0.0;

    public boolean beamBreakFront = false;
    public boolean beamBreakBack = false;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Confiure the PID for the flywheel. */
  public default void configureFlywheelPID(
      double kP, double kI, double kD, double kFF, double staticGain) {}

  /** Sets the target speed for the flywheel PID. */
  public default void setFlywheelSpeed(double rpm) {}

  /** Sets the target output for the flywheel motor. */
  public default void setFlywheelPercentOutput(double percent) {}

  /** Stops the flywheel motor(s). */
  public default void stopFlywheel() {}

  /** Configure the PID for the feed. */
  public default void configureFeedPID(double kP, double kI, double kD, double kFF) {}

  /** Sets the target speed for the feed PID. */
  public default void setFeedSpeed(double rpm) {}

  /** Sets the target output for the feed motor. */
  public default void setFeedPercentOutput(double percent) {}

  /** Enables or disables brake mode on the flywheel. */
  public default void setFlywheelBrakeMode(boolean enable) {}

  /** Enables or disables brake mode on the feed. */
  public default void setFeedBrakeMode(boolean enable) {}

  /** Stops the feed motor. */
  public default void stopFeed() {}

  /** Runs th eopen loop at the specified voltage. */
  public default void setVoltage(double volts) {}
}
