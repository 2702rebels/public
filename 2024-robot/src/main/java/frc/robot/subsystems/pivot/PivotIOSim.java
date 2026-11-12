package frc.robot.subsystems.pivot;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {
  private final DCMotorSim motorSim = new DCMotorSim(DCMotor.getNEO(1), 9 * 5 * (44 / 15), 0);

  private double appliedVolts = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    motorSim.update(Constants.cyclePeriodSeconds);

    inputs.measuredPositionRadians = motorSim.getAngularPositionRotations();
    inputs.measuredVelocityRadiansPerSecond = motorSim.getAngularVelocityRPM();
    inputs.appliedVolts = appliedVolts;
  }
}
