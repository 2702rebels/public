package frc.robot.subsystems.intake;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableNumber;
import org.littletonrobotics.junction.Logger;

public class Intake extends SubsystemBase {
  private final IntakeIO io;
  private final IntakeIOInputsAutoLogged inputs = new IntakeIOInputsAutoLogged();

  // spotless:off
  private static final LoggedTunableNumber intakeSpeed = new LoggedTunableNumber("Intake/Speed", 1.0);
  private static final LoggedTunableNumber intakeSpeedAuto = new LoggedTunableNumber("Intake/Speed Auto", 1.0);
  // spotless:on

  private boolean intaking = false;

  public Intake(IntakeIO io) {
    this.io = io;
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Intake/Inputs", inputs);
  }

  public void start() {
    io.setSpeed(DriverStation.isAutonomous() ? intakeSpeedAuto.get() : intakeSpeed.get());
    intaking = true;
  }

  public void stop() {
    io.setSpeed(0);
    intaking = false;
  }

  public void reverse() {
    io.setSpeed(-intakeSpeed.get());
    intaking = false;
  }

  public boolean isIntaking() {
    return intaking;
  }

  public boolean isBeamTripped() {
    return inputs.beamBreak;
  }
}
