package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
  private final ClimbIO io;
  private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

  // spotless:off
  private static final LoggedTunableNumber ratchetOnAngle = new LoggedTunableNumber("Climb/Ratchet On Angle", 0);
  private static final LoggedTunableNumber ratchetOffAngle = new LoggedTunableNumber("Climb/Ratchet Off Angle", 80);
  private static final LoggedTunableNumber hookRaisedPosition = new LoggedTunableNumber("Climb/Hook Raised Position", 55);
  private static final LoggedTunableNumber hookRetractedPosition = new LoggedTunableNumber("Climb/Hook Retracted Position", 10);
  private static final LoggedTunableNumber homeSpeed = new LoggedTunableNumber("Climb/Home Speed", -0.2);
  private static final LoggedTunableNumber tolerance = new LoggedTunableNumber("Climb/Tolerance", 2.0);
  private static final LoggedTunableNumber kP = new LoggedTunableNumber("Climb/Motor P", 0.1);
  private static final LoggedTunableNumber kI = new LoggedTunableNumber("Climb/Motor I", 0.0);
  private static final LoggedTunableNumber kD = new LoggedTunableNumber("Climb/Motor D", 0.0);
  // spotless:on

  private boolean isHomed = false;
  private double desiredPosition;

  public Climb(ClimbIO io) {
    this.io = io;
    this.io.configurePID(kP.get(), kI.get(), kD.get());
    engageRatchet();
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Climb/Inputs", inputs);

    final var callerId = hashCode();
    if (kP.hasChanged(callerId) || kI.hasChanged(callerId) || kD.hasChanged(callerId)) {
      System.out.println("[Climb] Motor PID parameters have changed. Reconfiguring them.");
      io.configurePID(kP.get(), kI.get(), kD.get());
    }
  }

  /** Determines whether limit switch closed. */
  @AutoLogOutput(key = "Climb/Limit Switch Closed")
  public boolean isLimitSwitchClosed() {
    return inputs.limitSwitchClosed;
  }

  /** Determines whether the climb has been homed. */
  @AutoLogOutput(key = "Climb/Homed")
  public boolean isHomed() {
    return isHomed;
  }

  /** Determines whether the climb reaches retract commanded position. */
  @AutoLogOutput(key = "Climb/Retracted")
  public boolean isAtRetractGoal() {
    return Math.abs(inputs.position - hookRetractedPosition.get()) < tolerance.get();
  }

  /** Determines whether the climb reaches raised commanded position. */
  @AutoLogOutput(key = "Climb/Raised")
  public boolean isAtRaisedGoal() {
    return Math.abs(inputs.position - hookRaisedPosition.get()) < tolerance.get();
  }

  /** Determines whethre the climb reaches manually commanded position. */
  public boolean isAtDesiredGoal() {
    return Math.abs(inputs.position - desiredPosition) < tolerance.get();
  }

  /** Turns climbing motor off. */
  public void stop() {
    io.setPercentOutput(0);
  }

  /** Runs climb motor in home mode. */
  public void home() {
    io.setPercentOutput(homeSpeed.get());
  }

  /** Engages ratchet. */
  public void engageRatchet() {
    io.setServoPosition(ratchetOnAngle.get());
  }

  /** Releases ratchet. */
  public void releaseRatchet() {
    io.setServoPosition(ratchetOffAngle.get());
  }

  /** Sets position to retracted. */
  public void setRetractPosition() {
    io.configurePID(kP.get(), kI.get(), kD.get());
    io.setPosition(hookRetractedPosition.get());
  }

  /** Sets position to relax the ratchet. */
  public void setRelaxPosition() {
    io.configurePID(kP.get(), kI.get(), kD.get());
    desiredPosition = Math.max(0, inputs.position - 4);
    io.setPosition(desiredPosition);
  }

  /** Sets position to raised. */
  public void setRaisePosition() {
    // note different kP here when raising hooks
    io.configurePID(kP.get() / 4, kI.get(), kD.get());
    io.setPosition(hookRaisedPosition.get());
  }

  /** Sets homed indicator and zeroes encoder if the limit switch is closed. */
  public void setHome() {
    // re-zero position when limit switch closes
    if (isLimitSwitchClosed()) {
      io.zeroEncoder();
      isHomed = true;
    }
  }
}
