package frc.robot.lib;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;

public class SwerveModuleTelemetry {
  private static final double kFastUpdateFrequencyHz = 250.0;

  private final StatusSignal<Temperature> driveTemperatureSignal;
  private final StatusSignal<Current> driveSupplyCurrentSignal;
  private final StatusSignal<AngularVelocity> driveVelocitySignal;
  private final StatusSignal<Double> driveClosedLoopReferenceSignal;

  private final StatusSignal<Temperature> steerTemperatureSignal;
  private final StatusSignal<Current> steerSupplyCurrentSignal;
  private final StatusSignal<Angle> steerPositionSignal;
  private final StatusSignal<Double> steerClosedLoopReferenceSignal;

  public SwerveModuleTelemetry(CommonTalon drive, CommonTalon steer) {

    driveTemperatureSignal = drive.getDeviceTemp();
    driveSupplyCurrentSignal = drive.getSupplyCurrent();
    driveVelocitySignal = drive.getVelocity();
    driveClosedLoopReferenceSignal = drive.getClosedLoopReference();

    PhoenixUtil.registerSignals(
        drive,
        kFastUpdateFrequencyHz,
        driveTemperatureSignal,
        driveSupplyCurrentSignal,
        driveVelocitySignal,
        driveClosedLoopReferenceSignal);

    steerTemperatureSignal = steer.getDeviceTemp();
    steerSupplyCurrentSignal = steer.getSupplyCurrent();
    steerPositionSignal = steer.getPosition();
    steerClosedLoopReferenceSignal = steer.getClosedLoopReference();

    PhoenixUtil.registerSignals(
        drive,
        kFastUpdateFrequencyHz,
        steerTemperatureSignal,
        steerSupplyCurrentSignal,
        steerPositionSignal,
        steerClosedLoopReferenceSignal);
  }

  @Logged(name = "Drive/Temperature", importance = Importance.CRITICAL)
  public double getDriveMotorTemperature() {
    return driveTemperatureSignal.getValue().in(Celsius);
  }

  @Logged(name = "Drive/SupplyCurrentAmps", importance = Importance.CRITICAL)
  public double getDriveSupplyCurrentAmps() {
    return driveSupplyCurrentSignal.getValue().in(Amps);
  }

  @Logged(name = "Drive/MotorVelocity", importance = Importance.DEBUG)
  public AngularVelocity getDriveMotorVelocity() {
    return driveVelocitySignal.getValue();
  }

  @Logged(name = "Drive/MotorClosedLoopReference", importance = Importance.DEBUG)
  public double getDriveMotorClosedLoopReference() {
    return driveClosedLoopReferenceSignal.getValue();
  }

  @Logged(name = "Steer/Temperature", importance = Importance.CRITICAL)
  public double geSteerMotorTemperature() {
    return steerTemperatureSignal.getValue().in(Celsius);
  }

  @Logged(name = "Steer/SupplyCurrentAmps", importance = Importance.CRITICAL)
  public double getSteerSupplyCurrentAmps() {
    return steerSupplyCurrentSignal.getValue().in(Amps);
  }

  @Logged(name = "Steer/MotorPosition", importance = Importance.DEBUG)
  public Angle getSteerMotorPosition() {
    return steerPositionSignal.getValue();
  }

  @Logged(name = "Steer/MotorClosedLoopReference", importance = Importance.DEBUG)
  public double getSteerMotorClosedLoopReference() {
    return steerClosedLoopReferenceSignal.getValue();
  }
}
