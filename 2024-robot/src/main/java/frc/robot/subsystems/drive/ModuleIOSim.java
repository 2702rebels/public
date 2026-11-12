package frc.robot.subsystems.drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

/**
 * Physics sim implementation of module IO.
 *
 * <p>Uses two flywheel sims for the drive and turn motors, with the absolute position initialized
 * to a random value. The flywheel sims are not physically accurate, but provide a decent
 * approximation for the behavior of the module.
 */
public class ModuleIOSim implements ModuleIO {
  private final DCMotorSim driveSim = new DCMotorSim(DCMotor.getNEO(1), 1.0, 0.025);
  private final DCMotorSim turnSim = new DCMotorSim(DCMotor.getNEO(1), 1.0, 0.004);

  private double driveAppliedVolts = 0.0;
  private double turnAppliedVolts = 0.0;

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    driveSim.update(Constants.cyclePeriodSeconds);
    turnSim.update(Constants.cyclePeriodSeconds);

    inputs.drivePositionMeters = driveSim.getAngularPositionRotations();
    inputs.driveVelocityMetersPerSec = driveSim.getAngularPositionRotations();
    inputs.driveAppliedVolts = driveAppliedVolts;
    inputs.driveCurrentAmps = Math.abs(driveSim.getCurrentDrawAmps());

    inputs.turnPosition = new Rotation2d(turnSim.getAngularPositionRad());
    inputs.turnVelocityRadPerSec = turnSim.getAngularVelocityRadPerSec();
    inputs.turnAppliedVolts = turnAppliedVolts;
    inputs.turnCurrentAmps = Math.abs(turnSim.getCurrentDrawAmps());

    inputs.odometryDrivePositionsMeters = new double[] {inputs.drivePositionMeters};
    inputs.odometryTurnPositions = new Rotation2d[] {inputs.turnPosition};
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    driveSim.setInputVoltage(driveAppliedVolts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnAppliedVolts = MathUtil.clamp(volts, -12.0, 12.0);
    turnSim.setInputVoltage(turnAppliedVolts);
  }
}
