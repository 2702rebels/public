package frc.robot.lib;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import java.util.function.Consumer;

public class LoggedTalonFXS extends TalonFXS {
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Current> statorCurrentSignal;
  private final StatusSignal<Temperature> temperatureSignal;
  private final StatusSignal<Voltage> motorVoltageSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Angle> positionSignal;

  public LoggedTalonFXS(int deviceId, CANBus canbus) {
    this(deviceId, canbus, null);
  }

  public LoggedTalonFXS(int deviceId, CANBus canbus, Consumer<TalonFXSConfiguration> configurator) {
    super(deviceId, canbus);

    final var config = new TalonFXSConfiguration();
    configure(config);

    // apply external configuration if specified
    if (configurator != null) {
      configurator.accept(config);
    }

    PhoenixUtil.config(this, config);

    velocitySignal = getVelocity();
    positionSignal = getPosition();
    temperatureSignal = getDeviceTemp();
    motorVoltageSignal = getMotorVoltage();
    supplyCurrentSignal = getSupplyCurrent();
    statorCurrentSignal = getStatorCurrent();

    PhoenixUtil.registerSignals(
        this,
        velocitySignal,
        positionSignal,
        temperatureSignal,
        motorVoltageSignal,
        supplyCurrentSignal,
        statorCurrentSignal);
  }

  protected void configure(TalonFXSConfiguration config) {}

  @Logged(name = "Temperature", importance = Importance.INFO)
  public double getTemperatureSignal() {
    return temperatureSignal.getValue().in(Celsius);
  }

  @Logged(name = "AppliedVolts", importance = Importance.INFO)
  public double getAppliedVolts() {
    return motorVoltageSignal.getValue().in(Volts);
  }

  @Logged(name = "StatorCurrentAmps", importance = Importance.INFO)
  public double getStatorCurrentAmps() {
    return statorCurrentSignal.getValue().in(Amps);
  }

  @Logged(name = "SupplyCurrentAmps", importance = Importance.INFO)
  public double getSupplyCurrentAmps() {
    return supplyCurrentSignal.getValue().in(Amps);
  }

  @Logged(name = "VelocityRPM", importance = Importance.INFO)
  public double getVelocityRPM() {
    return velocitySignal.getValue().in(RPM);
  }

  @Logged(name = "PositionRotations", importance = Importance.INFO)
  public double getPositionRotations() {
    return positionSignal.getValue().in(Rotations);
  }
}
