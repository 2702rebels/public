package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.lib.CANduit;
import frc.robot.lib.EasyCRT;
import frc.robot.lib.EasyCRT.CRTStatus;
import frc.robot.lib.EasyCRTConfig;
import frc.robot.lib.EnergyReporter;
import frc.robot.lib.LoggedTalonFX;
import frc.robot.lib.Misc;
import frc.robot.lib.Tunable;
import java.util.Optional;
import java.util.function.BooleanSupplier;
import java.util.function.Supplier;

public final class Turret extends SubsystemBase implements EnergyReporter {
  // MAKE SURE TO UPDATE THESE VALUES AFTER TUNING
  // DO NOT USE THESE VALUES DIRECTLY IN PERIODIC CODE,
  // USE THEM TO INITIALIZE TUNABLE VALUES AND CONTROLLERS
  private static final double __kP0 = 100;
  private static final double __kD0 = 2;

  private static final double __kS = 1.5;
  private static final double __kA = 0.07;
  private static final double __kV = 0.05;
  private static final double __kP = 50;
  private static final double __kD = 3.3;
  private static final double __kMaxVelocity = 0; // rps
  private static final double __kMaxAcceleration = 0; // rps/s
  private static final double __kJerk = 0;

  private static final double __kSetpointManual = 0;

  @Logged(name = "Motor", importance = Importance.CRITICAL)
  private final LoggedTalonFX motor;

  @Logged(name = "Setpoint", importance = Importance.CRITICAL)
  private Angle setpoint = Degrees.zero();

  @Logged(name = "CrtAngle", importance = Importance.DEBUG)
  private Angle crtAngle = Degrees.zero();

  @Logged(name = "CrtStatus", importance = Importance.CRITICAL)
  private CRTStatus crtStatus = CRTStatus.NOT_ATTEMPTED;

  @Logged(name = "Offset/Mean", importance = Importance.DEBUG)
  private Angle offsetMean = Degrees.zero();

  @Logged(name = "Offset/Stdev", importance = Importance.DEBUG)
  private Angle offsetStdev = Degrees.zero();

  private static final int kOffsetEstimatorSize = 10;
  private final CircularBuffer<Double> offsetEstimator = new CircularBuffer<>(kOffsetEstimatorSize);

  private final MedianFilter encoder1Filter = new MedianFilter(5);
  private final MedianFilter encoder2Filter = new MedianFilter(5);

  private final MotionMagicExpoTorqueCurrentFOC controlProfiled;
  private final PositionTorqueCurrentFOC controlSimple;
  private long controlProfiledTimestamp = -1;

  private final CANduit.PWMInput encoder1;
  private final CANduit.PWMInput encoder2;
  private final EasyCRT crt;

  // indicates that the subsystem has been initialized by determining the position of the turret
  private boolean initialized = false;

  // external turrent position supplier in tracking mode
  private Supplier<Angle> trackingAngleSupplier = () -> Degrees.zero();

  private static final double kTurretTeeth = 218.0;
  private static final double kEncoder1Ratio = kTurretTeeth / 17.0;
  private static final double kEncoder2Ratio = kTurretTeeth / 19.0;

  // THESE OFFSETS MUST BE UPDATED IF ENCODER GEARS ARE RESET
  //
  // 1. zero the turret with a zeroing block facing the rear of the robot,
  //    make sure the cord is in a neutral position and not wrapped around
  // 2. read the encoder values in the AdvantageScope and set these offsets
  //    to the corresponding sign-inverted values
  private static final Angle kEncoder1Offset = Degrees.of(-234.4836);
  private static final Angle kEncoder2Offset = Degrees.of(-57.853828);

  // min/max safe CRT range: these are dependent on the gear geometry
  // and do NOT represent the safe range of motion
  private static final Angle kMinRange = Degrees.of(-260.0);
  private static final Angle kMaxRange = Degrees.of(260.0);

  private static final double kStatorCurrentLimit = 60.0;
  private static final double kConversionFactor = kTurretTeeth * 3.0 / 22.0;
  private static final double kMinPosition = -190; // degrees
  private static final double kMaxPosition = 200.0; // degrees
  private static final double kMinOverlap = -360 - kMinPosition;
  private static final double kMaxOverlap = 360 - kMaxPosition;
  private static final double kTolerance = 5.0; // degrees
  private static final double kLongRangeArc = 30.0;

  // time to retain the profiled PID motion before switching to simple PID
  private static final long kControlProfiledRetentionTime = 100000; // microseconds

  // Expo kV and kA characterize the profile not gains
  // https://v6.docs.ctr-electronics.com/en/latest/docs/api-reference/device-specific/talonfx/motion-magic.html#motion-magic-expo
  //
  // max velocity (rps) = min(supply voltage / kV, cruise velocity)
  // max acceleration (rps/s) = supply voltage / kA
  //
  // For example,
  // kV = 0.03 at 12V -> max velocity = 400 rps
  // kA = 0.04 at 12V -> max acceleration = 300 rps/s
  //
  // SMALLER VALUES REPRESENT HIGHER MAX VELOCITY / ACCELERATION
  private static final double __kVExpo = 12.0 / (3.0 * kConversionFactor); // 3 rps
  private static final double __kAExpo = __kVExpo * 0.2; // 200ms to max velocity

  /** Convert turret angular position to motor position in rotations. */
  private static double toMotorPosition(Angle value) {
    return value.in(Rotations) * kConversionFactor;
  }

  // duty-cycle used to run initialization sequence
  private double initDutyCycle = 0.05;

  private final BooleanSupplier isAtSetpointDebounced;

  @Logged(name = "MasterOverride", importance = Importance.CRITICAL)
  public boolean masterOverride = false;

  public Turret(CANBus canbus, CANduit canduit) {
    motor =
        new LoggedTalonFX(
            Constants.CanivoreID.MOTOR_TURRET,
            canbus,
            (config) -> {
              config.CurrentLimits.StatorCurrentLimit = kStatorCurrentLimit;
              config.CurrentLimits.StatorCurrentLimitEnable = true;
              config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
              config.MotorOutput.ControlTimesyncFreqHz = 100;
              config.ClosedLoopGeneral.ContinuousWrap = false;

              config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
              config.SoftwareLimitSwitch.ForwardSoftLimitThreshold =
                  toMotorPosition(Degrees.of(kMaxPosition));

              config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;
              config.SoftwareLimitSwitch.ReverseSoftLimitThreshold =
                  toMotorPosition(Degrees.of(kMinPosition));

              config.Slot0.kP = __kP0;
              config.Slot0.kD = __kD0;

              config.Slot1.kS = __kS;
              config.Slot1.kA = __kA;
              config.Slot1.kV = __kV;
              config.Slot1.kP = __kP;
              config.Slot1.kD = __kD;

              config.MotionMagic.MotionMagicCruiseVelocity = __kMaxVelocity;
              config.MotionMagic.MotionMagicAcceleration = __kMaxAcceleration;
              config.MotionMagic.MotionMagicJerk = __kJerk;
              config.MotionMagic.MotionMagicExpo_kV = __kVExpo;
              config.MotionMagic.MotionMagicExpo_kA = __kAExpo;
            });

    controlSimple =
        new PositionTorqueCurrentFOC(0).withSlot(0).withUseTimesync(true).withUpdateFreqHz(0);

    controlProfiled =
        new MotionMagicExpoTorqueCurrentFOC(0)
            .withSlot(1)
            .withUseTimesync(true)
            .withUpdateFreqHz(0);

    // make configuration tunable (must be done after configuration is bound to the motor)
    Tunable.bind("Turret", motor);
    Tunable.bind1("Turret/Profiled", motor);
    Tunable.bindMotionProfile("Turret/Motion", motor);

    encoder1 = canduit.createPWMInput(Constants.CANduitID.TURRET_ENCODER1);
    encoder2 = canduit.createPWMInput(Constants.CANduitID.TURRET_ENCODER2);
    crt =
        new EasyCRT(
            new EasyCRTConfig(this::getEncoder1Angle, this::getEncoder2Angle)
                .withEncoderRatios(kEncoder1Ratio, kEncoder2Ratio)
                .withAbsoluteEncoderOffsets(kEncoder1Offset, kEncoder2Offset)
                .withMechanismRange(kMinRange, kMaxRange)
                .withMatchTolerance(Degrees.of(kTolerance)));

    // adjust initialization direction based on the presumed position
    initDutyCycle *= (getPosition().in(Degrees) >= 0 ? -1 : 1);

    isAtSetpointDebounced = Misc.debounce(this::isAtSetpoint, 0.250, DebounceType.kFalling);
  }

  /**
   * Converts raw PWM input into encoder position in rotations for REV through-borer encoder V1.
   * https://www.revrobotics.com/rev-11-1271/
   */
  private static double getEncoderRotations(CANduit.PWMInput encoder) {
    // 0°: 1μs, 360°: 1024μs
    //
    // REV encoders were found to be not agreeing to the specification,
    // of those tested reported periods were 1010μs and 1034μs;
    // we have also observed that pulse width does not go above 1025μs as per spec,
    // when the reported period is higher than 1025μs;
    //
    // below formula seems to handle both situations reasonably well

    // subtract min pulse width from period/pulse width to get range starting at zero
    final double v = Math.max(encoder.getPulseWidth() - 1000, 0);
    final double p = Math.min(encoder.getPeriod() - 1000, 1024000); // nanoseconds
    return MathUtil.inputModulus(v / p, 0, 1);
  }

  private static final double kEncoderRatioDiff = 1.0 / (kEncoder1Ratio - kEncoder2Ratio);

  private static Optional<Angle> getEstimatedTurretAngle(Angle encoder1, Angle encoder2) {
    // let n be the turret rotation, then encoder1 is (218/17)n and encoder2 is (218/19)n
    // difference between encoders = (218/17 - 218/19)n = (436/323)n
    // therefore, turret rotation can be derived from encoder difference as
    // n = 323 * (encoder1 - encoder2) / 436

    // adjust encoders by their respective offsets and convert to rotations
    final var encoder1rot =
        MathUtil.inputModulus(encoder1.plus(kEncoder1Offset).in(Rotations), 0, 1);
    final var encoder2rot =
        MathUtil.inputModulus(encoder2.plus(kEncoder2Offset).in(Rotations), 0, 1);

    final var encoderDiff = MathUtil.inputModulus(encoder1rot - encoder2rot, 0, 1);
    final var n = kEncoderRatioDiff * encoderDiff;

    // predict encoder1 reading from the derived turret rotation
    final var encoder1expected = MathUtil.inputModulus(n * kEncoder1Ratio, 0, 1);

    // encoder difference wraps every 323/436 rotations of the turret,
    // prediction will match if encoder difference has not yet wrapped;
    //
    // delta between prediction and actual encoder1 reading should be
    // either 0 or 0.5; the former indicates we are within the first wrap,
    // the latter indicates we must offset by one wrap
    //
    // here we assume that we are always within +/- wrap of the "zero" position
    // and subtract one wrap worth of rotation, since we treat CW and CCW as
    // opposite signs
    //
    // otherwise ignore the reading since it is likely garbage
    final var delta = Math.abs(encoder1expected - encoder1rot);

    if (delta < 0.015) {
      return Optional.of(Rotations.of(n));
    }

    if (Math.abs(delta - 0.5) < 0.015) {
      return Optional.of(Rotations.of(n - kEncoderRatioDiff));
    }

    return Optional.empty();
  }

  /** Logic to estimate turret angle using a history of CRT readings. */
  private void estimateTurretAngle(Angle estimatedAngle) {
    // record CRT angle for telemetry/debugging
    crtAngle = estimatedAngle;

    final var position = getPosition();
    final var offsetInDegrees =
        MathUtil.inputModulus(crtAngle.in(Degrees) - position.in(Degrees), -180, 180);

    offsetEstimator.addLast(offsetInDegrees);

    if (offsetEstimator.size() >= kOffsetEstimatorSize) {
      double mean = 0;
      for (int i = 0; i < kOffsetEstimatorSize; ++i) {
        final var v = offsetEstimator.get(i);
        mean += v;
      }
      mean /= kOffsetEstimatorSize;

      double stdev = 0;
      for (int i = 0; i < kOffsetEstimatorSize; ++i) {
        final var v = offsetEstimator.get(i);
        stdev += Math.pow(v - mean, 2);
      }
      stdev = Math.sqrt(stdev / kOffsetEstimatorSize);

      // record offset values for telemetry/debugging
      offsetMean = Degrees.of(mean);
      offsetStdev = Degrees.of(stdev);

      // reset turret motor position to the derived angle when the offset stabilizes
      if (!initialized && stdev < 2 * kTolerance) {
        motor.resetPosition(toMotorPosition(crtAngle));
        initialized = true;
      }
    }
  }

  @Logged(name = "Encoder1/Angle", importance = Importance.CRITICAL)
  public Angle getEncoder1Angle() {
    return Rotations.of(encoder1Filter.calculate(getEncoderRotations(encoder1)));
  }

  @Logged(name = "Encoder2/Angle", importance = Importance.CRITICAL)
  public Angle getEncoder2Angle() {
    return Rotations.of(encoder2Filter.calculate(getEncoderRotations(encoder2)));
  }

  @Logged(name = "Encoder1/Configured", importance = Importance.CRITICAL)
  public boolean isEncoder1Configured() {
    return encoder1.isConfigured();
  }

  @Logged(name = "Encoder2/Configured", importance = Importance.CRITICAL)
  public boolean isEncoder2Configured() {
    return encoder2.isConfigured();
  }

  @Logged(name = "HasSensors", importance = Importance.CRITICAL)
  public boolean hasSensors() {
    return encoder1.isConfigured()
        && encoder2.isConfigured()
        && encoder1.hasValue()
        && encoder2.hasValue();
  }

  @Override
  public void periodic() {
    if (!initialized) {
      // ensure both encoders are configured, this function is nop if the encoder
      // is already configured and retries configuration otherwise
      encoder1.configure(false);
      encoder2.configure(false);

      // ensure both encoders are reporting valid readings
      if (hasSensors()) {
        crtStatus = crt.getLastStatus();
        crt.getAngleOptional().ifPresent(this::estimateTurretAngle);
      }
    } else {
      if (masterOverride || kSetpointManualOverride.get()) {
        setAngle(Degrees.of(kSetpointManual.get()));
      } else {
        setAngle(trackingAngleSupplier.get());
      }

      final var timestamp = RobotController.getTime();

      // use profiled control for long range motion;
      // simple PID for short bursts and keeping the turret in place
      var useProfiledMotion = Math.abs(getPosition().minus(setpoint).in(Degrees)) > kLongRangeArc;

      // retain profiled motion for short amount of time to allow it to
      // complete before switching to simple PID control
      if (useProfiledMotion) {
        controlProfiledTimestamp = timestamp;
      } else {
        if (controlProfiledTimestamp > 0
            && timestamp - controlProfiledTimestamp < kControlProfiledRetentionTime) {
          useProfiledMotion = true;
        } else {
          controlProfiledTimestamp = -1;
        }
      }

      motor.setControl(
          useProfiledMotion
              ? controlProfiled.withPosition(toMotorPosition(setpoint))
              : controlSimple.withPosition(toMotorPosition(setpoint)));
    }
  }

  public void setAngle(Angle value) {
    final var currentDegrees = getPosition().in(Degrees);
    var targetDegrees = MathUtil.clamp(value.in(Degrees), kMinPosition, kMaxPosition);

    // unwrap target position to avoid a full turret sweep near +-180 degrees
    if (targetDegrees >= kMaxOverlap && currentDegrees < -90) {
      targetDegrees -= 360.0;
    } else if (targetDegrees <= kMinOverlap && currentDegrees > 90) {
      targetDegrees += 360.0;
    }

    setpoint = Degrees.of(targetDegrees);
  }

  @Logged(name = "Initialized", importance = Importance.CRITICAL)
  public boolean isInitialized() {
    return initialized;
  }

  /** Determines whether the subsystem is at its commanded setpoint. */
  @Logged(name = "AtSetpoint", importance = Importance.CRITICAL)
  public boolean isAtSetpoint() {
    return MathUtil.isNear(setpoint.in(Degrees), getPosition().in(Degrees), kTolerance);
  }

  @Logged(name = "IsReady", importance = Importance.CRITICAL)
  public boolean isReady() {
    final var currentPosition = getPosition().in(Degrees);
    final var currentSetpoint = setpoint.in(Degrees);

    // if setpoint is too far away (i.e. rotating around) then return false
    // otherwise, use debounced logic
    if (Math.abs(currentSetpoint - currentPosition) > kLongRangeArc) {
      return false;
    }

    return isAtSetpointDebounced.getAsBoolean();
  }

  @Logged(name = "Position", importance = Importance.CRITICAL)
  public Angle getPosition() {
    return motor.getMotorPosition().div(kConversionFactor);
  }

  /** Initializes turret position. */
  public Command initialize() {
    return run(() -> motor.set(initDutyCycle))
        .until(this::isInitialized)
        .withTimeout(Seconds.of(5));
  }

  /** Determines whether initialization sequence should be run. */
  public boolean shouldInitialize() {
    return !initialized && hasSensors();
  }

  public void setTrackingSupplier(Supplier<Angle> supplier) {
    trackingAngleSupplier = supplier;
  }

  @Override
  public double getTotalSupplyCurrent() {
    return Math.abs(motor.getSupplyCurrentAmps());
  }

  /** Runs self-check. */
  public Command selfCheck() {
    return Commands.sequence(
            Commands.print("Turret self-check started"),
            Commands.waitUntil(this::isInitialized),
            Commands.print("Turret initialized"),
            runOnce(() -> kSetpointManualOverride.set(true)),
            Commands.print("Turret to 0"),
            run(() -> kSetpointManual.set(0)).withTimeout(Seconds.of(2)),
            Commands.print("Turret to 90"),
            run(() -> kSetpointManual.set(90)).withTimeout(Seconds.of(2)),
            Commands.print("Turret to 180"),
            run(() -> kSetpointManual.set(180)).withTimeout(Seconds.of(2)),
            Commands.print("Turret to -90"),
            run(() -> kSetpointManual.set(-90)).withTimeout(Seconds.of(2)),
            Commands.print("Turret to 0"),
            run(() -> kSetpointManual.set(0)).withTimeout(Seconds.of(2)),
            runOnce(() -> kSetpointManualOverride.set(false)),
            Commands.print("Turret self-check completed"))
        .withName("TurretSelfCheck");
  }

  // spotless:off
  private static Tunable.DoubleValue kSetpointManual = Tunable.value("Turret/Manual", __kSetpointManual, true);
  private static Tunable.BooleanValue kSetpointManualOverride = Tunable.value("Turret/ManualOverride", false, true);
  // spotless:on
}
