package frc.robot.lib;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.ctre.phoenix6.hardware.traits.CommonTalon;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/** CTRE Phoenix 6 helper utilities. */
public final class PhoenixUtil {
  private static final Alert deviceInitFailure =
      new Alert("CTRE: device initialization error", AlertType.kError);

  // default signal update frequency
  private static final double kUpdateFrequencyHz = 100.0;

  private PhoenixUtil() {
    throw new AssertionError();
  }

  /** CANivore signals registered for a synchronized refresh */
  private static BaseStatusSignal[] registeredSignals = new BaseStatusSignal[0];

  private static void registerSignals(BaseStatusSignal... signals) {
    final var newSignals = new BaseStatusSignal[registeredSignals.length + signals.length];
    System.arraycopy(registeredSignals, 0, newSignals, 0, registeredSignals.length);
    System.arraycopy(signals, 0, newSignals, registeredSignals.length, signals.length);
    registeredSignals = newSignals;
  }

  /** Optimizes CAN bus utilization. */
  public static void optimizeBusUtilization(CommonTalon talon) {
    run("OptimizeBusUtilization", talon.getDeviceID(), () -> talon.optimizeBusUtilization());
  }

  /**
   * Registers a set of CANivore signals for a synchronized refresh with the specified update
   * frequency for the device.
   */
  public static void registerSignals(
      CommonTalon talon, double updateFrequencyHz, BaseStatusSignal... signals) {
    run(
        "SetSignalsUpdateFrequency",
        talon.getDeviceID(),
        () -> BaseStatusSignal.setUpdateFrequencyForAll(updateFrequencyHz, signals));
    registerSignals(signals);
  }

  /**
   * Registers a set of CANivore signals for a synchronized refresh with the default update
   * frequency for the device.
   */
  public static void registerSignals(CommonTalon talon, BaseStatusSignal... signals) {
    registerSignals(talon, kUpdateFrequencyHz, signals);
  }

  /** Refreshes all registered signals. Must be invoked from robot periodic. */
  public static void refreshSignals() {
    if (registeredSignals.length > 0) {
      BaseStatusSignal.refreshAll(registeredSignals);
    }
  }

  /**
   * Configures a {@link TalonFX} controller with error checking.
   *
   * @param talonFXS {@link TalonFX} controller to configure.
   * @param config The configuration to apply.
   * @return {@code true} if success ({@link StatusCode#isOK()}), {@code false} otherwise.
   */
  public static boolean config(TalonFX talonFX, TalonFXConfiguration config) {
    return run("Configure", talonFX.getDeviceID(), () -> talonFX.getConfigurator().apply(config));
  }

  /**
   * Configures a {@link TalonFXS} controller with error checking.
   *
   * @param talonFXS {@link TalonFXS} controller to configure.
   * @param config The configuration to apply.
   * @return {@code true} if success ({@link StatusCode#isOK()}), {@code false} otherwise.
   */
  public static boolean config(TalonFXS talonFXS, TalonFXSConfiguration config) {
    return run("Configure", talonFXS.getDeviceID(), () -> talonFXS.getConfigurator().apply(config));
  }

  /**
   * Runs a Phoenix 6 API call and checks for errors. Will try up to 5 times if the target API call
   * fails.
   *
   * @param name The name of the API call.
   * @param deviceId The identifier of the device the call is relevant to.
   * @param target The target call to run.
   * @return {@code true} if success ({@link StatusCode#isOK()}), {@code false} otherwise.
   */
  public static boolean run(String name, int deviceId, Supplier<StatusCode> target) {
    return run(name, deviceId, target, 5);
  }

  /**
   * Runs a Phoenix 6 API call and checks for errors.
   *
   * @param name The name of the API call.
   * @param deviceId The identifier of the device the call is relevant to.
   * @param target The target call to run.
   * @param maxTries The number of times to try the call before failing. {@code 1} only runs the
   *     call once.
   * @return {@code true} if success ({@link StatusCode#isOK()}), {@code false} otherwise.
   */
  public static boolean run(String name, int deviceId, Supplier<StatusCode> target, int maxTries) {
    String results = "";
    for (var i = 0; i < maxTries; ++i) {
      final var result = target.get();
      if (result.isOK()) return true;
      results += (results.isEmpty() ? "" : ", ") + result.name();
    }

    deviceInitFailure.set(true);
    DriverStation.reportError(
        "[PhoenixUtil] (ID " + deviceId + ") \"" + name + "\": " + results, false);

    return false;
  }
}
