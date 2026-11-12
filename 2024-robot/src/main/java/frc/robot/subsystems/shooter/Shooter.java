package frc.robot.subsystems.shooter;

import edu.wpi.first.units.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.util.LoggedTunableNumber;
import java.util.OptionalDouble;
import org.littletonrobotics.junction.Logger;

public class Shooter extends SubsystemBase {
  private final ShooterIO io;
  private final ShooterIOInputsAutoLogged inputs = new ShooterIOInputsAutoLogged();

  // spotless:off
  private static final LoggedTunableNumber kP  = new LoggedTunableNumber("Shooter/Flywheel P", 0.0001);
  private static final LoggedTunableNumber kI  = new LoggedTunableNumber("Shooter/Flywheel I", 0.0);
  private static final LoggedTunableNumber kD  = new LoggedTunableNumber("Shooter/Flywheel D", 0.0);
  private static final LoggedTunableNumber kFF = new LoggedTunableNumber("Shooter/Flywheel FF", 0.000183); // 0.000183
  private static final LoggedTunableNumber kS  = new LoggedTunableNumber("Shooter/Flywheel kS", 0.2265);
  private static final LoggedTunableNumber atSpeedAllowance = new LoggedTunableNumber("Shooter/AtSpeedAllowance", 100);
  private static final LoggedTunableNumber reverseSpeed = new LoggedTunableNumber("Shooter/Reverse Speed", -0.2);
  private static final LoggedTunableNumber shooterSpeedFactor = new LoggedTunableNumber("Shooter/SpeedFactor", 1.0);
  private static final LoggedTunableNumber feedSpeedOutput = new LoggedTunableNumber("Shooter/Feed Speed Output", 0.3);
  // spotless:on

  private final SysIdRoutine sysId;
  private boolean feeding = false;
  private OptionalDouble frontBeamTrippedAt = OptionalDouble.empty();

  public Shooter(ShooterIO io) {
    this.io = io;
    this.io.configureFlywheelPID(kP.get(), kI.get(), kD.get(), kFF.get(), kS.get());

    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                Units.Seconds.of(15),
                state -> Logger.recordOutput("Shooter/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Units.Volts)), null, this));
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Shooter/Inputs", inputs);

    final var callerId = hashCode();
    if (kP.hasChanged(callerId)
        || kI.hasChanged(callerId)
        || kD.hasChanged(callerId)
        || kFF.hasChanged(callerId)
        || kS.hasChanged(callerId)) {
      System.out.println(
          "[Shooter] Flywheel velocity PID parameters have changed. Reconfiguring them.");
      io.configureFlywheelPID(kP.get(), kI.get(), kD.get(), kFF.get(), kS.get());
    }

    if (isFrontBeamTripped() && frontBeamTrippedAt.isEmpty()) {
      frontBeamTrippedAt = OptionalDouble.of(Timer.getFPGATimestamp());
    }

    if (!isFrontBeamTripped() && frontBeamTrippedAt.isPresent()) {
      frontBeamTrippedAt = OptionalDouble.empty();
    }
  }

  /** Runs the open loop at the specified voltage. */
  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  /** Returns a command to run a quasistatic test in the specified direction. */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  /** Returns a command to run a dynamic test in the specified direction. */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public boolean isAtSpeed(double rpm) {
    return inputs.flywheelVelocity > rpm - atSpeedAllowance.get()
        && inputs.flywheelVelocity < rpm + atSpeedAllowance.get() * 2;
  }

  public boolean hasGamePiece() {
    return inputs.beamBreakFront || inputs.beamBreakBack;
  }

  public boolean isFrontBeamTripped() {
    return inputs.beamBreakFront;
  }

  public boolean isFrontBeamTrippedAndTimeout() {
    final var timestamp = Timer.getFPGATimestamp();
    return isFrontBeamTripped()
        && frontBeamTrippedAt.isPresent()
        && (timestamp - frontBeamTrippedAt.getAsDouble()) > 0.08;
  }

  public boolean isBackBeamTripped() {
    return inputs.beamBreakBack;
  }

  public boolean isFeeding() {
    return feeding;
  }

  public boolean isFeedLive() {
    return inputs.feedVelocity >= 1000;
  }

  public void setSpeed(double speed) {
    io.setFlywheelSpeed(speed * shooterSpeedFactor.get());
  }

  public void feedOut() {
    io.setFeedPercentOutput(feedSpeedOutput.get());
  }

  public void feedReverse() {
    io.setFeedPercentOutput(-0.2);
  }

  public void feedForward() {
    feedForward(0.3);
  }

  public void feedForward(double percent) {
    io.setFeedPercentOutput(0.2);
    feeding = true;
  }

  public void stopFeed() {
    io.stopFeed();
    feeding = false;
  }

  /** Stops shooter and feed. */
  public void stop() {
    stopFeed();
    io.stopFlywheel();
  }

  /** Enables or disables brake mode on the flywheel. */
  public void setFlywheelBrakeMode(boolean enable) {
    io.setFlywheelBrakeMode(enable);
  }

  /** Runs flywheel in reverse to keep notes from popping out. */
  public void reverseFlywheel() {
    io.setFlywheelPercentOutput(reverseSpeed.get());
  }

  public void percentFlywheel(double percent) {
    io.setFlywheelPercentOutput(percent);
  }
}
