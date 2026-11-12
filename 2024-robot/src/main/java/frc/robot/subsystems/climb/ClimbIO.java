package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {
  @AutoLog
  public static class ClimbIOInputs {
    public double position = 0.0;
    public double appliedVolts = 0.0;
    public double currentAmps = 0.0;
    public double velocity = 0.0;

    public boolean limitSwitchClosed = false;
  }

  public default void updateInputs(ClimbIOInputs inputs) {}

  public default void zeroEncoder() {}

  public default void setPercentOutput(double speed) {}

  public default void setPosition(double position) {}

  public default void configurePID(double kP, double kI, double kD) {}

  public default void setServoPosition(double degrees) {}
}
