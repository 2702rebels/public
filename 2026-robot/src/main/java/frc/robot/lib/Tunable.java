package frc.robot.lib;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.hardware.TalonFXS;
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
 * <p>Idea borrowed from:
 * https://github.com/Greater-Rochester-Robotics/Reefscape2025-340/blob/main/src/main/java/org/team340/lib/util/Tunable.java
 */
public final class Tunable {
  /** Indicates that competition mode is enabled and only critical values should be made tunable. */
  private static final boolean kCompetitionMode =
      Constants.DeploymentMode == Constants.Deployment.kCompetition;

  private Tunable() {
    throw new AssertionError();
  }

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
    private final double defaultValue;

    private DoubleValue(
        String name, double defaultValue, boolean critical, Optional<DoubleConsumer> onChange) {
      this.defaultValue = defaultValue;

      if (kCompetitionMode && !critical) {
        entry = null;
        return;
      }

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
      return entry != null ? entry.get() : defaultValue;
    }

    public void set(double value) {
      if (entry != null) {
        entry.set(value);
      }
    }

    @Override
    public double getAsDouble() {
      return get();
    }

    @Override
    public void close() {
      if (entry != null) {
        entry.close();
      }
    }
  }

  /** Tunable double array value. */
  public static final class DoubleArrayValue implements Supplier<double[]>, AutoCloseable {
    private final DoubleArrayEntry entry;
    private final double[] defaultValue;

    private DoubleArrayValue(
        String name,
        double[] defaultValue,
        boolean critical,
        Optional<Consumer<double[]>> onChange) {
      this.defaultValue = defaultValue;

      if (kCompetitionMode && !critical) {
        entry = null;
        return;
      }

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
      return entry != null ? entry.get() : defaultValue;
    }

    public void set(double[] value) {
      if (entry != null) {
        entry.set(value);
      }
    }

    @Override
    public void close() {
      if (entry != null) {
        entry.close();
      }
    }
  }

  /** Tunable boolean value. */
  public static final class BooleanValue implements BooleanSupplier, AutoCloseable {
    private final BooleanEntry entry;
    private final boolean defaultValue;

    private BooleanValue(
        String name, boolean defaultValue, boolean critical, Optional<BooleanConsumer> onChange) {
      this.defaultValue = defaultValue;

      if (kCompetitionMode && !critical) {
        entry = null;
        return;
      }

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
      return entry != null ? entry.get() : defaultValue;
    }

    public void set(boolean value) {
      if (entry != null) {
        entry.set(value);
      }
    }

    @Override
    public boolean getAsBoolean() {
      return get();
    }

    @Override
    public void close() {
      if (entry != null) {
        entry.close();
      }
    }
  }

  /** Tunable boolean array value. */
  public static final class BooleanArrayValue implements Supplier<boolean[]>, AutoCloseable {
    private final BooleanArrayEntry entry;
    private final boolean[] defaultValue;

    private BooleanArrayValue(
        String name,
        boolean[] defaultValue,
        boolean critical,
        Optional<Consumer<boolean[]>> onChange) {
      this.defaultValue = defaultValue;

      if (kCompetitionMode && !critical) {
        entry = null;
        return;
      }

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
      return entry != null ? entry.get() : defaultValue;
    }

    public void set(boolean[] value) {
      if (entry != null) {
        entry.set(value);
      }
    }

    @Override
    public void close() {
      if (entry != null) {
        entry.close();
      }
    }
  }

  /** Tunable string value. */
  public static final class StringValue implements Supplier<String>, AutoCloseable {
    private final StringEntry entry;
    private final String defaultValue;

    private StringValue(
        String name, String defaultValue, boolean critical, Optional<Consumer<String>> onChange) {
      this.defaultValue = defaultValue;

      if (kCompetitionMode && !critical) {
        entry = null;
        return;
      }

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
      return entry != null ? entry.get() : defaultValue;
    }

    public void set(String value) {
      if (entry != null) {
        entry.set(value);
      }
    }

    @Override
    public void close() {
      if (entry != null) {
        entry.close();
      }
    }
  }

  /** Tunable string array value. */
  public static final class StringArrayValue implements Supplier<String[]>, AutoCloseable {
    private final StringArrayEntry entry;
    private final String[] defaultValue;

    private StringArrayValue(
        String name,
        String[] defaultValue,
        boolean critical,
        Optional<Consumer<String[]>> onChange) {
      this.defaultValue = defaultValue;

      if (kCompetitionMode && !critical) {
        entry = null;
        return;
      }

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
      return entry != null ? entry.get() : defaultValue;
    }

    public void set(String[] value) {
      if (entry != null) {
        entry.set(value);
      }
    }

    @Override
    public void close() {
      if (entry != null) {
        entry.close();
      }
    }
  }

  /** Tunable struct value. */
  public static final class StructValue<T> implements Supplier<T>, AutoCloseable {
    private final StructEntry<T> entry;
    private final T defaultValue;

    private StructValue(
        String name,
        Struct<T> struct,
        T defaultValue,
        boolean critical,
        Optional<Consumer<T>> onChange) {
      this.defaultValue = defaultValue;

      if (kCompetitionMode && !critical) {
        entry = null;
        return;
      }

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
      return entry != null ? entry.get() : defaultValue;
    }

    public void set(T value) {
      if (entry != null) {
        entry.set(value);
      }
    }

    @Override
    public void close() {
      if (entry != null) {
        entry.close();
      }
    }
  }

  /** Tunable struct array value. */
  public static final class StructArrayValue<T> implements Supplier<T[]>, AutoCloseable {
    private final StructArrayEntry<T> entry;
    private final T[] defaultValue;

    private StructArrayValue(
        String name,
        Struct<T> struct,
        T[] defaultValue,
        Boolean critical,
        Optional<Consumer<T[]>> onChange) {
      this.defaultValue = defaultValue;

      if (kCompetitionMode && !critical) {
        entry = null;
        return;
      }

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
      return entry != null ? entry.get() : defaultValue;
    }

    public void set(T[] value) {
      if (entry != null) {
        entry.set(value);
      }
    }

    @Override
    public void close() {
      if (entry != null) {
        entry.close();
      }
    }
  }

  /** Creates non-critical tunable double value with the specified default. */
  public static DoubleValue value(String name, double defaultValue) {
    return value(name, defaultValue, false);
  }

  /** Creates tunable double value with the specified default. */
  public static DoubleValue value(String name, double defaultValue, boolean critical) {
    return new DoubleValue(name, defaultValue, critical, Optional.empty());
  }

  /** Creates non-critical tunable double value with the specified default and change handler. */
  public static DoubleValue value(String name, double defaultValue, DoubleConsumer onChange) {
    return value(name, defaultValue, false, onChange);
  }

  /** Creates tunable double value with the specified default and change handler. */
  public static DoubleValue value(
      String name, double defaultValue, boolean critical, DoubleConsumer onChange) {
    return new DoubleValue(name, defaultValue, critical, Optional.of(onChange));
  }

  /** Creates non-critical tunable double array value with the specified default. */
  public static DoubleArrayValue value(String name, double[] defaultValue) {
    return value(name, defaultValue, false);
  }

  /** Creates tunable double array value with the specified default. */
  public static DoubleArrayValue value(String name, double[] defaultValue, boolean critical) {
    return new DoubleArrayValue(name, defaultValue, critical, Optional.empty());
  }

  /**
   * Creates non-critical tunable double array value with the specified default and change handler.
   */
  public static DoubleArrayValue value(
      String name, double[] defaultValue, Consumer<double[]> onChange) {
    return value(name, defaultValue, onChange);
  }

  /** Creates tunable double array value with the specified default and change handler. */
  public static DoubleArrayValue value(
      String name, double[] defaultValue, boolean critical, Consumer<double[]> onChange) {
    return new DoubleArrayValue(name, defaultValue, critical, Optional.of(onChange));
  }

  /** Creates non-critical tunable boolean value with the specified default. */
  public static BooleanValue value(String name, boolean defaultValue) {
    return value(name, defaultValue, false);
  }

  /** Creates tunable boolean value with the specified default. */
  public static BooleanValue value(String name, boolean defaultValue, boolean critical) {
    return new BooleanValue(name, defaultValue, critical, Optional.empty());
  }

  /** Creates non-critical tunable boolean value with the specified default and change handler. */
  public static BooleanValue value(String name, boolean defaultValue, BooleanConsumer onChange) {
    return value(name, defaultValue, false, onChange);
  }

  /** Creates tunable boolean value with the specified default and change handler. */
  public static BooleanValue value(
      String name, boolean defaultValue, boolean critical, BooleanConsumer onChange) {
    return new BooleanValue(name, defaultValue, critical, Optional.of(onChange));
  }

  /** Creates non-critical tunable boolean array value with the specified default. */
  public static BooleanArrayValue value(String name, boolean[] defaultValue) {
    return value(name, defaultValue, false);
  }

  /** Creates tunable boolean array value with the specified default. */
  public static BooleanArrayValue value(String name, boolean[] defaultValue, boolean critical) {
    return new BooleanArrayValue(name, defaultValue, critical, Optional.empty());
  }

  /**
   * Creates non-critical tunable boolean array value with the specified default and change handler.
   */
  public static BooleanArrayValue value(
      String name, boolean[] defaultValue, Consumer<boolean[]> onChange) {
    return value(name, defaultValue, false, onChange);
  }

  /** Creates tunable boolean array value with the specified default and change handler. */
  public static BooleanArrayValue value(
      String name, boolean[] defaultValue, boolean critical, Consumer<boolean[]> onChange) {
    return new BooleanArrayValue(name, defaultValue, critical, Optional.of(onChange));
  }

  /** Creates non-critical tunable string value with the specified default. */
  public static StringValue value(String name, String defaultValue) {
    return value(name, defaultValue, false);
  }

  /** Creates tunable string value with the specified default. */
  public static StringValue value(String name, String defaultValue, boolean critical) {
    return new StringValue(name, defaultValue, critical, Optional.empty());
  }

  /** Creates non-critical tunable string value with the specified default and change handler. */
  public static StringValue value(String name, String defaultValue, Consumer<String> onChange) {
    return value(name, defaultValue, false, onChange);
  }

  /** Creates tunable string value with the specified default and change handler. */
  public static StringValue value(
      String name, String defaultValue, boolean critical, Consumer<String> onChange) {
    return new StringValue(name, defaultValue, critical, Optional.of(onChange));
  }

  /** Creates non-critical tunable string array value with the specified default. */
  public static StringArrayValue value(String name, String[] defaultValue) {
    return value(name, defaultValue, false);
  }

  /** Creates tunable string array value with the specified default. */
  public static StringArrayValue value(String name, String[] defaultValue, boolean critical) {
    return new StringArrayValue(name, defaultValue, critical, Optional.empty());
  }

  /**
   * Creates non-critical tunable string array value with the specified default and change handler.
   */
  public static StringArrayValue value(
      String name, String[] defaultValue, Consumer<String[]> onChange) {
    return value(name, defaultValue, false, onChange);
  }

  /** Creates tunable string array value with the specified default and change handler. */
  public static StringArrayValue value(
      String name, String[] defaultValue, boolean critical, Consumer<String[]> onChange) {
    return new StringArrayValue(name, defaultValue, critical, Optional.of(onChange));
  }

  /** Creates non-critical tunable struct value with the specified default. */
  public static <T> StructValue<T> value(String name, Struct<T> struct, T defaultValue) {
    return value(name, struct, defaultValue, false);
  }

  /** Creates tunable struct value with the specified default. */
  public static <T> StructValue<T> value(
      String name, Struct<T> struct, T defaultValue, boolean critical) {
    return new StructValue<T>(name, struct, defaultValue, critical, Optional.empty());
  }

  /** Creates non-critical tunable struct value with the specified default and change handler. */
  public static <T> StructValue<T> value(
      String name, Struct<T> struct, T defaultValue, Consumer<T> onChange) {
    return value(name, struct, defaultValue, false, onChange);
  }

  /** Creates tunable struct value with the specified default and change handler. */
  public static <T> StructValue<T> value(
      String name, Struct<T> struct, T defaultValue, boolean critical, Consumer<T> onChange) {
    return new StructValue<T>(name, struct, defaultValue, critical, Optional.of(onChange));
  }

  /** Creates non-critical tunable struct array value with the specified default. */
  public static <T> StructArrayValue<T> value(String name, Struct<T> struct, T[] defaultValue) {
    return value(name, struct, defaultValue, false);
  }

  /** Creates tunable struct array value with the specified default. */
  public static <T> StructArrayValue<T> value(
      String name, Struct<T> struct, T[] defaultValue, boolean critical) {
    return new StructArrayValue<T>(name, struct, defaultValue, critical, Optional.empty());
  }

  /**
   * Creates non-critical tunable struct array value with the specified default and change handler.
   */
  public static <T> StructArrayValue<T> value(
      String name, Struct<T> struct, T[] defaultValue, Consumer<T[]> onChange) {
    return value(name, struct, defaultValue, false);
  }

  /** Creates tunable struct array value with the specified default and change handler. */
  public static <T> StructArrayValue<T> value(
      String name, Struct<T> struct, T[] defaultValue, boolean critical, Consumer<T[]> onChange) {
    return new StructArrayValue<T>(name, struct, defaultValue, critical, Optional.of(onChange));
  }

  /** Makes bound {@link PIDController} tunable. */
  public static void bind(String name, PIDController value) {
    bind(name, value, false);
  }

  /** Makes bound {@link PIDController} tunable. */
  public static void bind(String name, PIDController value, boolean critical) {
    value(name + "/kP", value.getP(), critical, v -> value.setP(v));
    value(name + "/kI", value.getI(), critical, v -> value.setI(v));
    value(name + "/kD", value.getD(), critical, v -> value.setD(v));
    value(name + "/IZone", value.getIZone(), critical, v -> value.setIZone(v));
    value(name + "/Tolerance", value.getErrorTolerance(), critical, v -> value.setTolerance(v));
  }

  /** Makes bound {@link ProfiledPIDController} tunable. */
  public static void bind(String name, ProfiledPIDController value) {
    bind(name, value, false);
  }

  /** Makes bound {@link ProfiledPIDController} tunable. */
  public static void bind(String name, ProfiledPIDController value, boolean critical) {
    value(name + "/kP", value.getP(), critical, v -> value.setP(v));
    value(name + "/kI", value.getI(), critical, v -> value.setI(v));
    value(name + "/kD", value.getD(), critical, v -> value.setD(v));
    value(name + "/IZone", value.getIZone(), critical, v -> value.setIZone(v));
    value(name + "/Tolerance", value.getPositionTolerance(), critical, v -> value.setTolerance(v));
    value(
        name + "/MaxV",
        value.getConstraints().maxVelocity,
        critical,
        v ->
            value.setConstraints(
                new TrapezoidProfile.Constraints(v, value.getConstraints().maxAcceleration)));
    value(
        name + "/MaxA",
        value.getConstraints().maxAcceleration,
        critical,
        v ->
            value.setConstraints(
                new TrapezoidProfile.Constraints(value.getConstraints().maxVelocity, v)));
  }

  /** Makes bound {@link TalonFX} controller at closed loop slot 0 tunable. */
  public static void bind(String name, TalonFX talonFX) {
    bind(name, talonFX, false);
  }

  /** Makes bound {@link TalonFX} controller at closed loop slot 0 tunable. */
  public static void bind(String name, TalonFX talonFX, boolean critical) {
    final var config = new Slot0Configs();
    talonFX.getConfigurator().refresh(config);

    value(
        name + "/kP",
        config.kP,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kP = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kI",
        config.kI,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kI = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kD",
        config.kD,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kD = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kS",
        config.kS,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kS = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kV",
        config.kV,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kV = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kA",
        config.kA,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kA = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kG",
        config.kG,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kG = v;
          talonFX.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link TalonFX} controller at closed loop slot 1 tunable. */
  public static void bind1(String name, TalonFX talonFX) {
    bind1(name, talonFX, false);
  }

  /** Makes bound {@link TalonFX} controller at closed loop slot 1 tunable. */
  public static void bind1(String name, TalonFX talonFX, boolean critical) {
    final var config = new Slot1Configs();
    talonFX.getConfigurator().refresh(config);

    value(
        name + "/kP",
        config.kP,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kP = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kI",
        config.kI,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kI = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kD",
        config.kD,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kD = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kS",
        config.kS,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kS = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kV",
        config.kV,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kV = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kA",
        config.kA,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kA = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kG",
        config.kG,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kG = v;
          talonFX.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link TalonFX} controller at closed loop slot 2 tunable. */
  public static void bind2(String name, TalonFX talonFX) {
    bind1(name, talonFX, false);
  }

  /** Makes bound {@link TalonFX} controller at closed loop slot 2 tunable. */
  public static void bind2(String name, TalonFX talonFX, boolean critical) {
    final var config = new Slot2Configs();
    talonFX.getConfigurator().refresh(config);

    value(
        name + "/kP",
        config.kP,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kP = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kI",
        config.kI,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kI = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kD",
        config.kD,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kD = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kS",
        config.kS,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kS = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kV",
        config.kV,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kV = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kA",
        config.kA,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kA = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/kG",
        config.kG,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.kG = v;
          talonFX.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link TalonFX} controller Motion Magic tunable. */
  public static void bindMotionProfile(String name, TalonFX talonFX) {
    bindMotionProfile(name, talonFX, false);
  }

  /** Makes bound {@link TalonFX} controller Motion Magic tunable. */
  public static void bindMotionProfile(String name, TalonFX talonFX, boolean critical) {
    final var config = new MotionMagicConfigs();
    talonFX.getConfigurator().refresh(config);

    value(
        name + "/Velocity",
        config.MotionMagicCruiseVelocity,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.MotionMagicCruiseVelocity = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/Acceleration",
        config.MotionMagicAcceleration,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.MotionMagicAcceleration = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/Jerk",
        config.MotionMagicJerk,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.MotionMagicJerk = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/Expo kV",
        config.MotionMagicExpo_kV,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.MotionMagicExpo_kV = v;
          talonFX.getConfigurator().apply(config);
        });
    value(
        name + "/Expo kA",
        config.MotionMagicExpo_kA,
        critical,
        v -> {
          talonFX.getConfigurator().refresh(config);
          config.MotionMagicExpo_kA = v;
          talonFX.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link TalonFXS} controller at closed loop slot 0 tunable. */
  public static void bind(String name, TalonFXS talonFXS) {
    bind(name, talonFXS, false);
  }

  /** Makes bound {@link TalonFXS} controller at closed loop slot 0 tunable. */
  public static void bind(String name, TalonFXS talonFXS, boolean critical) {
    final var config = new Slot0Configs();
    talonFXS.getConfigurator().refresh(config);

    value(
        name + "/kP",
        config.kP,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kP = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kI",
        config.kI,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kI = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kD",
        config.kD,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kD = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kS",
        config.kS,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kS = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kV",
        config.kV,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kV = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kA",
        config.kA,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kA = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kG",
        config.kG,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kG = v;
          talonFXS.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link TalonFXS} controller at closed loop slot 1 tunable. */
  public static void bind1(String name, TalonFXS talonFXS) {
    bind1(name, talonFXS, false);
  }

  /** Makes bound {@link TalonFXS} controller at closed loop slot 1 tunable. */
  public static void bind1(String name, TalonFXS talonFXS, boolean critical) {
    final var config = new Slot1Configs();
    talonFXS.getConfigurator().refresh(config);

    value(
        name + "/kP",
        config.kP,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kP = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kI",
        config.kI,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kI = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kD",
        config.kD,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kD = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kS",
        config.kS,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kS = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kV",
        config.kV,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kV = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kA",
        config.kA,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kA = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kG",
        config.kG,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kG = v;
          talonFXS.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link TalonFXS} controller at closed loop slot 1 tunable. */
  public static void bind2(String name, TalonFXS talonFXS) {
    bind2(name, talonFXS, false);
  }

  /** Makes bound {@link TalonFXS} controller at closed loop slot 1 tunable. */
  public static void bind2(String name, TalonFXS talonFXS, boolean critical) {
    final var config = new Slot2Configs();
    talonFXS.getConfigurator().refresh(config);

    value(
        name + "/kP",
        config.kP,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kP = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kI",
        config.kI,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kI = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kD",
        config.kD,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kD = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kS",
        config.kS,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kS = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kV",
        config.kV,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kV = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kA",
        config.kA,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kA = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/kG",
        config.kG,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.kG = v;
          talonFXS.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link TalonFXS} controller Motion Magic tunable. */
  public static void bindMotionProfile(String name, TalonFXS talonFXS) {
    bindMotionProfile(name, talonFXS, false);
  }

  /** Makes bound {@link TalonFXS} controller Motion Magic tunable. */
  public static void bindMotionProfile(String name, TalonFXS talonFXS, boolean critical) {
    final var config = new MotionMagicConfigs();
    talonFXS.getConfigurator().refresh(config);

    value(
        name + "/Velocity",
        config.MotionMagicCruiseVelocity,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.MotionMagicCruiseVelocity = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/Acceleration",
        config.MotionMagicAcceleration,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.MotionMagicAcceleration = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/Jerk",
        config.MotionMagicJerk,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.MotionMagicJerk = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/Expo kV",
        config.MotionMagicExpo_kV,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.MotionMagicExpo_kV = v;
          talonFXS.getConfigurator().apply(config);
        });
    value(
        name + "/Expo kA",
        config.MotionMagicExpo_kA,
        critical,
        v -> {
          talonFXS.getConfigurator().refresh(config);
          config.MotionMagicExpo_kA = v;
          talonFXS.getConfigurator().apply(config);
        });
  }

  /** Makes bound {@link ArmFeedforward} tunable. */
  public static void bind(String name, ArmFeedforward value) {
    bind(name, value, false);
  }

  /** Makes bound {@link ArmFeedforward} tunable. */
  public static void bind(String name, ArmFeedforward value, boolean critical) {
    value(name + "/kS", value.getKs(), critical, v -> value.setKs(v));
    value(name + "/kG", value.getKg(), critical, v -> value.setKg(v));
    value(name + "/kV", value.getKv(), critical, v -> value.setKv(v));
    value(name + "/kA", value.getKa(), critical, v -> value.setKa(v));
  }

  /** Makes bound {@link ElevatorFeedforward} tunable. */
  public static void bind(String name, ElevatorFeedforward value) {
    bind(name, value, false);
  }

  /** Makes bound {@link ElevatorFeedforward} tunable. */
  public static void bind(String name, ElevatorFeedforward value, boolean critical) {
    value(name + "/kS", value.getKs(), critical, v -> value.setKs(v));
    value(name + "/kG", value.getKg(), critical, v -> value.setKg(v));
    value(name + "/kV", value.getKv(), critical, v -> value.setKv(v));
    value(name + "/kA", value.getKa(), critical, v -> value.setKa(v));
  }
}
