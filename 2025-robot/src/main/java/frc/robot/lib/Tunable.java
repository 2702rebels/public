package frc.robot.lib;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.BooleanArrayEntry;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringArrayEntry;
import edu.wpi.first.networktables.StringEntry;
import edu.wpi.first.networktables.StructArrayEntry;
import edu.wpi.first.networktables.StructEntry;
import edu.wpi.first.util.function.BooleanConsumer;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.event.EventLoop;
import frc.robot.Constants;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Consumer;
import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

/**
 * Creates tunable parameters.
 *
 * <p>Borrowed from:
 * https://github.com/Greater-Rochester-Robotics/Reefscape2025-340/blob/main/src/main/java/org/team340/lib/util/Tunable.java
 */
public final class Tunable {
  private static final NetworkTable nt =
      NetworkTableInstance.getDefault().getTable(Constants.tunablePrefix);

  private static final EventLoop loop = new EventLoop();

  /** Must be invoked in the robot periodic. */
  public static void periodic() {
    loop.poll();
  }

  /** Tunable double value. */
  public static final class DoubleValue implements DoubleSupplier, AutoCloseable {
    private final DoubleEntry entry;

    private DoubleValue(String name, double defaultValue, Optional<DoubleConsumer> onChange) {
      entry = nt.getDoubleTopic(name).getEntry(defaultValue);
      entry.setDefault(defaultValue);

      if (onChange.isPresent()) {
        loop.bind(
            () -> {
              final var values = entry.readQueueValues();
              if (values.length > 0) {
                onChange.get().accept(values[values.length - 1]);
              }
            });
      }
    }

    public double get() {
      return entry.get();
    }

    public void set(double value) {
      entry.set(value);
    }

    @Override
    public double getAsDouble() {
      return get();
    }

    @Override
    public void close() {
      entry.close();
    }
  }

  /** Tunable double array value. */
  public static final class DoubleArrayValue implements Supplier<double[]>, AutoCloseable {
    private final DoubleArrayEntry entry;

    private DoubleArrayValue(
        String name, double[] defaultValue, Optional<Consumer<double[]>> onChange) {
      entry = nt.getDoubleArrayTopic(name).getEntry(defaultValue);
      entry.setDefault(defaultValue);

      if (onChange.isPresent()) {
        loop.bind(
            () -> {
              final var values = entry.readQueueValues();
              if (values.length > 0) {
                onChange.get().accept(values[values.length - 1]);
              }
            });
      }
    }

    public double[] get() {
      return entry.get();
    }

    public void set(double[] value) {
      entry.set(value);
    }

    @Override
    public void close() {
      entry.close();
    }
  }

  /** Tunable boolean value. */
  public static final class BooleanValue implements BooleanSupplier, AutoCloseable {
    private final BooleanEntry entry;

    private BooleanValue(String name, boolean defaultValue, Optional<BooleanConsumer> onChange) {
      entry = nt.getBooleanTopic(name).getEntry(defaultValue);
      entry.setDefault(defaultValue);

      if (onChange.isPresent()) {
        loop.bind(
            () -> {
              final var values = entry.readQueueValues();
              if (values.length > 0) {
                onChange.get().accept(values[values.length - 1]);
              }
            });
      }
    }

    public boolean get() {
      return entry.get();
    }

    public void set(boolean value) {
      entry.set(value);
    }

    @Override
    public boolean getAsBoolean() {
      return get();
    }

    @Override
    public void close() {
      entry.close();
    }
  }

  /** Tunable boolean array value. */
  public static final class BooleanArrayValue implements Supplier<boolean[]>, AutoCloseable {
    private final BooleanArrayEntry entry;

    private BooleanArrayValue(
        String name, boolean[] defaultValue, Optional<Consumer<boolean[]>> onChange) {
      entry = nt.getBooleanArrayTopic(name).getEntry(defaultValue);
      entry.setDefault(defaultValue);

      if (onChange.isPresent()) {
        loop.bind(
            () -> {
              final var values = entry.readQueueValues();
              if (values.length > 0) {
                onChange.get().accept(values[values.length - 1]);
              }
            });
      }
    }

    public boolean[] get() {
      return entry.get();
    }

    public void set(boolean[] value) {
      entry.set(value);
    }

    @Override
    public void close() {
      entry.close();
    }
  }

  /** Tunable string value. */
  public static final class StringValue implements Supplier<String>, AutoCloseable {
    private final StringEntry entry;

    private StringValue(String name, String defaultValue, Optional<Consumer<String>> onChange) {
      entry = nt.getStringTopic(name).getEntry(defaultValue);
      entry.setDefault(defaultValue);

      if (onChange.isPresent()) {
        loop.bind(
            () -> {
              final var values = entry.readQueueValues();
              if (values.length > 0) {
                onChange.get().accept(values[values.length - 1]);
              }
            });
      }
    }

    public String get() {
      return entry.get();
    }

    public void set(String value) {
      entry.set(value);
    }

    @Override
    public void close() {
      entry.close();
    }
  }

  /** Tunable string array value. */
  public static final class StringArrayValue implements Supplier<String[]>, AutoCloseable {
    private final StringArrayEntry entry;

    private StringArrayValue(
        String name, String[] defaultValue, Optional<Consumer<String[]>> onChange) {
      entry = nt.getStringArrayTopic(name).getEntry(defaultValue);
      entry.setDefault(defaultValue);

      if (onChange.isPresent()) {
        loop.bind(
            () -> {
              final var values = entry.readQueueValues();
              if (values.length > 0) {
                onChange.get().accept(values[values.length - 1]);
              }
            });
      }
    }

    public String[] get() {
      return entry.get();
    }

    public void set(String[] value) {
      entry.set(value);
    }

    @Override
    public void close() {
      entry.close();
    }
  }

  /** Tunable struct value. */
  public static final class StructValue<T> implements Supplier<T>, AutoCloseable {
    private final StructEntry<T> entry;

    private StructValue(
        String name, Struct<T> struct, T defaultValue, Optional<Consumer<T>> onChange) {
      entry = nt.getStructTopic(name, struct).getEntry(defaultValue);
      entry.setDefault(defaultValue);

      if (onChange.isPresent()) {
        loop.bind(
            () -> {
              final var values = entry.readQueueValues();
              if (values.length > 0) {
                onChange.get().accept(values[values.length - 1]);
              }
            });
      }
    }

    public T get() {
      return entry.get();
    }

    public void set(T value) {
      entry.set(value);
    }

    @Override
    public void close() {
      entry.close();
    }
  }

  /** Tunable struct array value. */
  public static final class StructArrayValue<T> implements Supplier<T[]>, AutoCloseable {
    private final StructArrayEntry<T> entry;

    private StructArrayValue(
        String name, Struct<T> struct, T[] defaultValue, Optional<Consumer<T[]>> onChange) {
      entry = nt.getStructArrayTopic(name, struct).getEntry(defaultValue);
      entry.setDefault(defaultValue);

      if (onChange.isPresent()) {
        loop.bind(
            () -> {
              final var values = entry.readQueueValues();
              if (values.length > 0) {
                onChange.get().accept(values[values.length - 1]);
              }
            });
      }
    }

    public T[] get() {
      return entry.get();
    }

    public void set(T[] value) {
      entry.set(value);
    }

    @Override
    public void close() {
      entry.close();
    }
  }

  /** Creates tunable double value with the specified default. */
  public static DoubleValue value(String name, double defaultValue) {
    return new DoubleValue(name, defaultValue, Optional.empty());
  }

  /** Creates tunable double value with the specified default and change handler. */
  public static DoubleValue value(String name, double defaultValue, DoubleConsumer onChange) {
    return new DoubleValue(name, defaultValue, Optional.of(onChange));
  }

  /** Creates tunable double array value with the specified default. */
  public static DoubleArrayValue value(String name, double[] defaultValue) {
    return new DoubleArrayValue(name, defaultValue, Optional.empty());
  }

  /** Creates tunable double array value with the specified default and change handler. */
  public static DoubleArrayValue value(
      String name, double[] defaultValue, Consumer<double[]> onChange) {
    return new DoubleArrayValue(name, defaultValue, Optional.of(onChange));
  }

  /** Creates tunable boolean value with the specified default. */
  public static BooleanValue value(String name, boolean defaultValue) {
    return new BooleanValue(name, defaultValue, Optional.empty());
  }

  /** Creates tunable boolean value with the specified default and change handler. */
  public static BooleanValue value(String name, boolean defaultValue, BooleanConsumer onChange) {
    return new BooleanValue(name, defaultValue, Optional.of(onChange));
  }

  /** Creates tunable boolean array value with the specified default. */
  public static BooleanArrayValue value(String name, boolean[] defaultValue) {
    return new BooleanArrayValue(name, defaultValue, Optional.empty());
  }

  /** Creates tunable boolean array value with the specified default and change handler. */
  public static BooleanArrayValue value(
      String name, boolean[] defaultValue, Consumer<boolean[]> onChange) {
    return new BooleanArrayValue(name, defaultValue, Optional.of(onChange));
  }

  /** Creates tunable string value with the specified default. */
  public static StringValue value(String name, String defaultValue) {
    return new StringValue(name, defaultValue, Optional.empty());
  }

  /** Creates tunable string value with the specified default and change handler. */
  public static StringValue value(String name, String defaultValue, Consumer<String> onChange) {
    return new StringValue(name, defaultValue, Optional.of(onChange));
  }

  /** Creates tunable string array value with the specified default. */
  public static StringArrayValue value(String name, String[] defaultValue) {
    return new StringArrayValue(name, defaultValue, Optional.empty());
  }

  /** Creates tunable string array value with the specified default and change handler. */
  public static StringArrayValue value(
      String name, String[] defaultValue, Consumer<String[]> onChange) {
    return new StringArrayValue(name, defaultValue, Optional.of(onChange));
  }

  /** Creates tunable struct value with the specified default. */
  public static <T> StructValue<T> value(String name, Struct<T> struct, T defaultValue) {
    return new StructValue<T>(name, struct, defaultValue, Optional.empty());
  }

  /** Creates tunable struct value with the specified default and change handler. */
  public static <T> StructValue<T> value(
      String name, Struct<T> struct, T defaultValue, Consumer<T> onChange) {
    return new StructValue<T>(name, struct, defaultValue, Optional.of(onChange));
  }

  /** Creates tunable struct array value with the specified default. */
  public static <T> StructArrayValue<T> value(String name, Struct<T> struct, T[] defaultValue) {
    return new StructArrayValue<T>(name, struct, defaultValue, Optional.empty());
  }

  /** Creates tunable struct array value with the specified default and change handler. */
  public static <T> StructArrayValue<T> value(
      String name, Struct<T> struct, T[] defaultValue, Consumer<T[]> onChange) {
    return new StructArrayValue<T>(name, struct, defaultValue, Optional.of(onChange));
  }

  /** Makes bound {@link PIDController} tunable. */
  public static void bind(String name, PIDController value) {
    value(name + "/kP", value.getP(), v -> value.setP(v));
    value(name + "/kI", value.getI(), v -> value.setI(v));
    value(name + "/kD", value.getD(), v -> value.setD(v));
    value(name + "/IZone", value.getIZone(), v -> value.setIZone(v));
    value(name + "/Tolerance", value.getErrorTolerance(), v -> value.setTolerance(v));
  }

  /** Makes bound {@link ProfiledPIDController} tunable. */
  public static void bind(String name, ProfiledPIDController value) {
    value(name + "/kP", value.getP(), v -> value.setP(v));
    value(name + "/kI", value.getI(), v -> value.setI(v));
    value(name + "/kD", value.getD(), v -> value.setD(v));
    value(name + "/IZone", value.getIZone(), v -> value.setIZone(v));
    value(name + "/Tolerance", value.getPositionTolerance(), v -> value.setTolerance(v));
    value(
        name + "/MaxV",
        value.getConstraints().maxVelocity,
        v ->
            value.setConstraints(
                new TrapezoidProfile.Constraints(v, value.getConstraints().maxAcceleration)));
    value(
        name + "/MaxA",
        value.getConstraints().maxAcceleration,
        v ->
            value.setConstraints(
                new TrapezoidProfile.Constraints(value.getConstraints().maxVelocity, v)));
  }

  /** Makes bound {@link TalonFX} controller at closed loop slot 0 tunable.. */
  public static void bind(String name, TalonFX talonFX) {
    final var config = new Slot0Configs();
    talonFX.getConfigurator().refresh(config);

    value(
        name + "/kP",
        config.kP,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kP = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kI",
        config.kI,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kI = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kD",
        config.kD,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kD = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kS",
        config.kS,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kS = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kV",
        config.kV,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kV = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kA",
        config.kA,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kA = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kG",
        config.kG,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kG = v;
          talonFX.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link TalonFX} controller Motion Magic tunable. */
  public static void bindMotionProfile(String name, TalonFX talonFX) {
    final var config = new MotionMagicConfigs();
    talonFX.getConfigurator().refresh(config);

    value(
        name + "/Velocity",
        config.MotionMagicCruiseVelocity,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.MotionMagicCruiseVelocity = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/Acceleration",
        config.MotionMagicAcceleration,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.MotionMagicAcceleration = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/Jerk",
        config.MotionMagicJerk,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.MotionMagicJerk = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/Expo kV",
        config.MotionMagicExpo_kV,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.MotionMagicExpo_kV = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/Expo kA",
        config.MotionMagicExpo_kA,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.MotionMagicExpo_kA = v;
          talonFX.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link TalonFXS} controller at closed loop slot 0 tunable.. */
  public static void bind(String name, TalonFXS talonFXS) {
    final var config = new Slot0Configs();
    talonFXS.getConfigurator().refresh(config);

    value(
        name + "/kP",
        config.kP,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kP = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kI",
        config.kI,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kI = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kD",
        config.kD,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kD = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kS",
        config.kS,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kS = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kV",
        config.kV,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kV = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kA",
        config.kA,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kA = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kG",
        config.kG,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kG = v;
          talonFXS.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link TalonFXS} controller Motion Magic tunable. */
  public static void bindMotionProfile(String name, TalonFXS talonFXS) {
    final var config = new MotionMagicConfigs();
    talonFXS.getConfigurator().refresh(config);

    value(
        name + "/Velocity",
        config.MotionMagicCruiseVelocity,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.MotionMagicCruiseVelocity = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/Acceleration",
        config.MotionMagicAcceleration,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.MotionMagicAcceleration = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/Jerk",
        config.MotionMagicJerk,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.MotionMagicJerk = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/Expo kV",
        config.MotionMagicExpo_kV,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.MotionMagicExpo_kV = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/Expo kA",
        config.MotionMagicExpo_kA,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.MotionMagicExpo_kA = v;
          talonFXS.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link SparkMax} controller at closed loop slot 0 tunable. */
  public static void bind(String name, SparkMax spark) {
    bind(name, spark, ClosedLoopSlot.kSlot0);
  }

  /** Makes bound {@link SparkMax} controller tunable. */
  public static void bind(String name, SparkMax spark, ClosedLoopSlot slot) {
    final var config = spark.configAccessor.closedLoop;
    value(
        name + "/kP",
        config.getP(slot),
        v -> {
          final var newConfig = new SparkMaxConfig();
          newConfig.closedLoop.p(v, slot);
          RevUtil.configEphemeral(spark, newConfig);
        });
    value(
        name + "/kI",
        config.getI(slot),
        v -> {
          final var newConfig = new SparkMaxConfig();
          newConfig.closedLoop.i(v, slot);
          RevUtil.configEphemeral(spark, newConfig);
        });
    value(
        name + "/kD",
        config.getD(slot),
        v -> {
          final var newConfig = new SparkMaxConfig();
          newConfig.closedLoop.d(v, slot);
          RevUtil.configEphemeral(spark, newConfig);
        });
    value(
        name + "/kF",
        config.getFF(slot),
        v -> {
          final var newConfig = new SparkMaxConfig();
          newConfig.closedLoop.velocityFF(v, slot);
          RevUtil.configEphemeral(spark, newConfig);
        });
    value(
        name + "/IZone",
        config.getIZone(slot),
        v -> {
          final var newConfig = new SparkMaxConfig();
          newConfig.closedLoop.iZone(v, slot);
          RevUtil.configEphemeral(spark, newConfig);
        });
    value(
        name + "/DFilter",
        config.getDFilter(slot),
        v -> {
          final var newConfig = new SparkMaxConfig();
          newConfig.closedLoop.dFilter(v, slot);
          RevUtil.configEphemeral(spark, newConfig);
        });
    value(
        name + "/MinOutput",
        config.getMinOutput(slot),
        v -> {
          final var newConfig = new SparkMaxConfig();
          newConfig.closedLoop.minOutput(v, slot);
          RevUtil.configEphemeral(spark, newConfig);
        });
    value(
        name + "/MaxOutput",
        config.getMaxOutput(slot),
        v -> {
          final var newConfig = new SparkMaxConfig();
          newConfig.closedLoop.maxOutput(v, slot);
          RevUtil.configEphemeral(spark, newConfig);
        });
    value(
        name + "/MaxVelocity",
        config.maxMotion.getMaxVelocity(slot),
        v -> {
          final var newConfig = new SparkMaxConfig();
          newConfig.closedLoop.maxMotion.maxVelocity(v, slot);
          RevUtil.configEphemeral(spark, newConfig);
        });
    value(
        name + "/MaxAcceleration",
        config.maxMotion.getMaxAcceleration(slot),
        v -> {
          final var newConfig = new SparkMaxConfig();
          newConfig.closedLoop.maxMotion.maxAcceleration(v, slot);
          RevUtil.configEphemeral(spark, newConfig);
        });
  }

  /** Makes bound {@link ArmFeedforward} tunable. */
  public static void bind(String name, ArmFeedforward value) {
    value(name + "/kS", value.getKs(), v -> value.setKs(v));
    value(name + "/kG", value.getKg(), v -> value.setKg(v));
    value(name + "/kV", value.getKv(), v -> value.setKv(v));
    value(name + "/kA", value.getKa(), v -> value.setKa(v));
  }

  /** Makes bound {@link ElevatorFeedforward} tunable. */
  public static void bind(String name, ElevatorFeedforward value) {
    value(name + "/kS", value.getKs(), v -> value.setKs(v));
    value(name + "/kG", value.getKg(), v -> value.setKg(v));
    value(name + "/kV", value.getKv(), v -> value.setKv(v));
    value(name + "/kA", value.getKa(), v -> value.setKa(v));
  }
}
