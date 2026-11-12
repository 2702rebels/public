package frc.robot.lib;

import com.revrobotics.REVLibError;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

/**
 * REVLib 2025 helper utilities.
 *
 * <p>Borrowed from:
 * https://github.com/Greater-Rochester-Robotics/Reefscape2025-340/blob/main/src/main/java/org/team340/lib/util/vendors/RevUtil.java
 */
public final class RevUtil {
  private RevUtil() {
    throw new AssertionError();
  }

  /**
   * Configures a Spark motor controller with error checking and a default state reset. Uses {@link
   * ResetMode#kResetSafeParameters} and {@link PersistMode#kPersistParameters}.
   *
   * @param spark The Spark to configure.
   * @param config The configuration to apply.
   * @return {@code true} if success ({@link REVLibError#kOk}), {@code false} otherwise.
   */
  public static boolean config(SparkMax spark, SparkMaxConfig config) {
    return run(
        "Configure",
        spark,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /**
   * Configures a Spark motor controller with error checking and a default state reset. Uses {@link
   * ResetMode#kResetSafeParameters} and {@link PersistMode#kPersistParameters}.
   *
   * @param spark The Spark to configure.
   * @param config The configuration to apply.
   * @return {@code true} if success ({@link REVLibError#kOk}), {@code false} otherwise.
   */
  public static boolean config(SparkFlex spark, SparkFlexConfig config) {
    return run(
        "Configure",
        spark,
        () ->
            spark.configure(
                config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters));
  }

  /**
   * Configures a Spark motor controller with error checking and no state reset. Uses {@link
   * ResetMode#kNoResetSafeParameters} and {@link PersistMode#kPersistParameters}.
   *
   * @param spark The Spark to configure.
   * @param config The configuration to apply.
   * @return {@code true} if success ({@link REVLibError#kOk}), {@code false} otherwise.
   */
  public static boolean configNoReset(SparkMax spark, SparkMaxConfig config) {
    return run(
        "Configure (No Reset)",
        spark,
        () ->
            spark.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  /**
   * Configures a Spark motor controller with error checking and no state reset. Uses {@link
   * ResetMode#kNoResetSafeParameters} and {@link PersistMode#kPersistParameters}.
   *
   * @param spark The Spark to configure.
   * @param config The configuration to apply.
   * @return {@code true} if success ({@link REVLibError#kOk}), {@code false} otherwise.
   */
  public static boolean configNoReset(SparkFlex spark, SparkFlexConfig config) {
    return run(
        "Configure (No Reset)",
        spark,
        () ->
            spark.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters));
  }

  /**
   * Configures a Spark motor controller ephemerally with error checking. Uses {@link
   * ResetMode#kNoResetSafeParameters} and {@link PersistMode#kNoPersistParameters}.
   *
   * @param spark The Spark to configure.
   * @param config The configuration to apply.
   * @return {@code true} if success ({@link REVLibError#kOk}), {@code false} otherwise.
   */
  public static boolean configEphemeral(SparkMax spark, SparkMaxConfig config) {
    return run(
        "Configure (Ephemeral)",
        spark,
        () ->
            spark.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  /**
   * Configures a Spark motor controller ephemerally with error checking. Uses {@link
   * ResetMode#kNoResetSafeParameters} and {@link PersistMode#kNoPersistParameters}.
   *
   * @param spark The Spark to configure.
   * @param config The configuration to apply.
   * @return {@code true} if success ({@link REVLibError#kOk}), {@code false} otherwise.
   */
  public static boolean configEphemeral(SparkFlex spark, SparkFlexConfig config) {
    return run(
        "Configure (Ephemeral)",
        spark,
        () ->
            spark.configure(
                config, ResetMode.kNoResetSafeParameters, PersistMode.kNoPersistParameters));
  }

  /**
   * Runs a REVLib API call and checks for errors. Will try up to 3 times if the target API call
   * fails.
   *
   * @param name The name of the API call.
   * @param device The device the call is relevant to.
   * @param target The target call to run.
   * @return {@code true} if success ({@link REVLibError#kOk}), {@code false} otherwise.
   */
  public static boolean run(String name, SparkLowLevel device, Supplier<REVLibError> target) {
    return run(name, device, target, 3);
  }

  /**
   * Runs a REVLib API call and checks for errors.
   *
   * @param name The name of the API call.
   * @param device The device the call is relevant to.
   * @param target The target call to run.
   * @param maxTries The number of times to try the call before failing. {@code 1} only runs the
   *     call once.
   * @return {@code true} if success ({@link REVLibError#kOk}), {@code false} otherwise.
   */
  public static boolean run(
      String name, SparkLowLevel device, Supplier<REVLibError> target, int maxTries) {
    String results = "";
    for (var i = 0; i < maxTries; ++i) {
      final var result = target.get();
      if (result.equals(REVLibError.kOk)) return true;
      results += (results.isEmpty() ? "" : ", ") + result.name();
    }

    DogLog.logFault("REV: device initialization error");
    DriverStation.reportError(
        "[RevUtil] "
            + device.getClass().getSimpleName()
            + " (ID "
            + device.getDeviceId()
            + ") \""
            + name
            + "\": "
            + results,
        false);

    return false;
  }
}
