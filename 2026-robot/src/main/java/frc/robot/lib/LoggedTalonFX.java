package frc.robot.lib;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Celsius;
import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import java.util.function.Consumer;

public class LoggedTalonFX extends TalonFX {
  private final StatusSignal<Temperature> temperatureSignal;
  private final StatusSignal<Double> dutyCycleSignal;
  private final StatusSignal<Current> supplyCurrentSignal;
  private final StatusSignal<Current> torqueCurrentSignal;
  private final StatusSignal<Angle> positionSignal;
  private final StatusSignal<AngularVelocity> velocitySignal;
  private final StatusSignal<Double> closedLoopReferenceSignal;
  private final double statorCurrentLimit;

  private static final double kFastUpdateFrequencyHz = 250.0;
  private static final double kSlowUpdateFrequencyHz = 50.0;

  public LoggedTalonFX(int deviceId, CANBus canbus) {
    this(deviceId, canbus, null);
  }

  public LoggedTalonFX(int deviceId, CANBus canbus, Consumer<TalonFXConfiguration> configurator) {
    super(deviceId, canbus);

    final var config = new TalonFXConfiguration();
    configure(config);

    // apply external configuration if specified
    if (configurator != null) {
      configurator.accept(config);
    }

    PhoenixUtil.config(this, config);

    // retain stator current to use in stall condition detection
    if (config.CurrentLimits.StatorCurrentLimitEnable) {
      statorCurrentLimit = config.CurrentLimits.StatorCurrentLimit;
    } else {
      throw new Error(
          "StatorCurrentLimit is not enabled for device with ID "
              + deviceId
              + " ("
              + getDescription()
              + ")");
    }

    // NOTE: it is important to register ALL signals that may be required for leader/follower
    // control with appropriate update frequencies, since `optimizeBusUtilization` call will
    // reduce all signals without explicit update frequency to 4Hz

    temperatureSignal = getDeviceTemp();
    supplyCurrentSignal = getSupplyCurrent();
    closedLoopReferenceSignal = getClosedLoopReference();

    PhoenixUtil.registerSignals(
        this,
        kSlowUpdateFrequencyHz,
        temperatureSignal,
        supplyCurrentSignal,
        closedLoopReferenceSignal);

    dutyCycleSignal = getDutyCycle();
    torqueCurrentSignal = getTorqueCurrent();
    positionSignal = getPosition();
    velocitySignal = getVelocity();

    PhoenixUtil.registerSignals(
        this,
        kFastUpdateFrequencyHz,
        dutyCycleSignal,
        torqueCurrentSignal,
        positionSignal,
        velocitySignal);

    PhoenixUtil.optimizeBusUtilization(this);
  }

  protected void configure(TalonFXConfiguration config) {}

  @Logged(name = "Temperature", importance = Importance.CRITICAL)
  public double getMotorTemperature() {
    return temperatureSignal.getValue().in(Celsius);
  }

  @Logged(name = "DutyCycle", importance = Importance.CRITICAL)
  public double getMotorDutyCycle() {
    return dutyCycleSignal.getValue();
  }

  @Logged(name = "TorqueCurrentAmps", importance = Importance.CRITICAL)
  public double getTorqueCurrentAmps() {
    return torqueCurrentSignal.getValue().in(Amps);
  }

  @Logged(name = "SupplyCurrentAmps", importance = Importance.CRITICAL)
  public double getSupplyCurrentAmps() {
    return supplyCurrentSignal.getValue().in(Amps);
  }

  @Logged(name = "MotorPosition", importance = Importance.DEBUG)
  public Angle getMotorPosition() {
    return positionSignal.getValue();
  }

  @Logged(name = "MotorVelocity", importance = Importance.DEBUG)
  public AngularVelocity getMotorVelocity() {
    return velocitySignal.getValue();
  }

  @Logged(name = "MotorClosedLoopReference", importance = Importance.DEBUG)
  public double getMotorClosedLoopReference() {
    return closedLoopReferenceSignal.getValue();
  }

  /** Detects stall condition of the motor. */
  public boolean isStalled() {
    return Math.abs(getTorqueCurrentAmps()) > statorCurrentLimit * 0.9
        && Math.abs(getMotorVelocity().in(RPM)) < 6;
  }

  /** Detects stall condition of the motor given the current limit. */
  public boolean isStalled(double currentLimitAmps) {
    return Math.abs(getTorqueCurrentAmps()) > currentLimitAmps
        && Math.abs(getMotorVelocity().in(RPM)) < 6;
  }

  /** Resets motor position. Note, that this is a blocking call and should be invoked sparingly. */
  public void resetPosition(double position) {
    PhoenixUtil.run("Reset Position", getDeviceID(), () -> setPosition(position));
  }
}
