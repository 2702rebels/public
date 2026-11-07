package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.filter.MedianFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.GamePhaseTracker.Phase;
import frc.robot.lib.GamePhaseTracker.PhaseInfo;
import frc.robot.lib.ShotCalculator;
import frc.robot.lib.SwerveTelemetry;
import frc.robot.lib.TimestampedCircularBuffer;
import frc.robot.lib.Tunable;
import frc.robot.lib.VisionState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Arrays;
import java.util.Optional;
import java.util.function.Supplier;

public class RobotState {
  private static RobotState instance;

  public static synchronized RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  public static enum PassingPoint {
    kDefault,
    kTrenchAuto,
    kTrenchLeft,
    kTrenchRight,
  }

  private static final long kVisionCutoff = 100 * 1000; // microseconds

  private static final double kStationaryVelocityTolerance = 0.1; // 10 cm/sec
  private static final double kStationarySpeedTolerance = 0.05; // 5 cm/sec
  private static final double kStationaryOmegaTolerance = Units.degreesToRadians(5); // 5 deg/sec
  private static final double kSlowSpeedTolerance = 1; // 1.0 m/sec
  private static final double kSlowOmegaTolerance = Units.degreesToRadians(60); // 60 deg/sec
  private static final double kRobotSlipThreshold = 0.3;
  private static final double kOdometryTrustThreshold = 0.8;
  private static final double kOdometryTrustPenalty = 0.05;
  private static final double kLowBatteryVoltage = 11.0;
  private static final double kLevelThreshold = 5.0; // degrees

  // shooting constants
  private static final Distance kMinScoreDistance = Meters.of(1.5);
  private static final Distance kMaxScoreDistance = Meters.of(6.0);
  private static final double kMaxScoreSpeed = 2.5; // m/s
  private static final Time kMechanismDelay = Seconds.of(0.15);

  // fudge factor (in seconds) to allow shooting to start before hub becomes active
  // captures both the mechanism delay (0.5s) and ToF (~1s)
  private static final double kTimeBeforeActiveFudge = 1.5;

  // fudget factor (in seconds) to allow shooting to extend after hub becomes inactive
  // captures both the hub counting extension (3s) and ToF (~1s)
  private static final double kTimeAfterActiveFudge = 1.5;

  // offset to the center of the turret in meters
  public static final Translation2d kShooterOffsetXY =
      new Translation2d(Meters.convertFrom(-6, Inches), Meters.convertFrom(-6, Inches));

  // offset to the zero of the turret from the robot heading
  public static final Rotation2d kTurretOffset = Rotation2d.fromDegrees(180);

  // std.dev for vision: we do trust vision so set these relatively low
  private static final Matrix<N3, N1> kVisionStdDevs =
      VecBuilder.fill(Units.inchesToMeters(5), Units.inchesToMeters(5), Units.degreesToRadians(10));

  // timestamped log of recently measured (within last second) chassis speeds
  private final TimestampedCircularBuffer<ChassisSpeeds> chassisSpeeds =
      new TimestampedCircularBuffer<>(50);

  // robot-centric acceleration tracking
  private final LinearFilter robotAx = LinearFilter.movingAverage(3);
  private final LinearFilter robotAy = LinearFilter.movingAverage(3);
  private final LinearFilter robotAw = LinearFilter.movingAverage(3);

  // bound drivetrain
  private CommandSwerveDrivetrain drivetrain;

  // external vision state supplier
  private Supplier<VisionState> visionStateSupplier;

  // external game phase supplier
  private Supplier<PhaseInfo> phaseInfoSupplier;

  private long visionPoseTimestamp = -1;
  private Pose2d visionPose = Pose2d.kZero;
  private Phase phase;
  private Time phaseElapsedTime = Seconds.zero();
  private Time phaseRemainingTime = Seconds.zero();
  private boolean hubActive;
  private Field.FieldZone fieldZone = Field.FieldZone.kAlliance;

  private final ShotCalculator shotCalculator;
  private final ShotCalculator passCalculator;

  private ShotCalculator.ShotParameters shotParameters =
      new ShotCalculator.ShotParameters(
          Translation2d.kZero, Meters.zero(), Meters.zero(), Seconds.zero(), 0, 0, 0, 0);

  private boolean shotPossible;

  private final MedianFilter shotVelocity = new MedianFilter(5);
  private final MedianFilter shotAngle = new MedianFilter(5);
  private final MedianFilter turretAngle = new MedianFilter(5);

  @Logged(name = "PassingPoint", importance = Importance.CRITICAL)
  private PassingPoint passingPoint = PassingPoint.kDefault;

  @Logged(name = "Shot/Velocity", importance = Importance.CRITICAL)
  public double getShotVelocity() {
    return shotVelocity.lastValue();
  }

  @Logged(name = "Shot/Distance", importance = Importance.CRITICAL)
  public double getShotDistance() {
    return shotParameters.virtualDistance().in(Meters);
  }

  @Logged(name = "Shot/ToF", importance = Importance.CRITICAL)
  public double getShotToF() {
    return shotParameters.tof().in(Seconds);
  }

  @Logged(name = "Shot/Angle", importance = Importance.CRITICAL)
  public double getShotAngle() {
    return shotAngle.lastValue();
  }

  @Logged(name = "Shot/Quality", importance = Importance.CRITICAL)
  public double getShotQuality() {
    return shotParameters.quality();
  }

  @Logged(name = "Shot/Iterations", importance = Importance.CRITICAL)
  public int getShotIterations() {
    return shotParameters.iterations();
  }

  @Logged(name = "Shot/Turret", importance = Importance.CRITICAL)
  public Angle getTurretAngle() {
    return Degrees.of(turretAngle.lastValue());
  }

  @Logged(name = "Shot/Possible", importance = Importance.CRITICAL)
  public boolean isShotPossible() {
    return shotPossible;
  }

  @Logged(name = "Shot/Suppressed", importance = Importance.CRITICAL)
  public boolean isShotSuppressed() {
    // suppression only works when FMS is attached
    if (!DriverStation.isFMSAttached()) {
      return false;
    }

    // no suppression while hub is active
    if (hubActive) {
      return false;
    }

    // fudge at before/after active phase
    return phaseElapsedTime.in(Seconds) > kTimeAfterActiveFudge
        && phaseRemainingTime.in(Seconds) > kTimeBeforeActiveFudge;
  }

  public Tunable.BooleanValue kVisionAssist = Tunable.value("VisionAssist", true, true);

  private RobotState() {
    if (Constants.Settings.ctreLoggingEnabled) {
      SignalLogger.setPath("/u/logs/ctre/");
      SignalLogger.start();
    }

    // initialize shot/pass calculator LUT
    shotCalculator = new ShotCalculator(kShooterOffsetXY);
    passCalculator = new ShotCalculator(kShooterOffsetXY);

    // accounts of adjustments made to the hood after LUT was constructed
    final var hoodDatum = -5.0;

    // shooting
    shotCalculator.putLUTEntry(Meters.of(1.25), 3050, hoodDatum + 10, 0.75);
    shotCalculator.putLUTEntry(Meters.of(1.50), 3000, hoodDatum + 10, 0.75);
    shotCalculator.putLUTEntry(Meters.of(1.80), 2900, hoodDatum + 15, 0.80);
    shotCalculator.putLUTEntry(Meters.of(2.00), 3000, hoodDatum + 15, 0.85);
    shotCalculator.putLUTEntry(Meters.of(2.50), 3150, hoodDatum + 20, 0.92);
    shotCalculator.putLUTEntry(Meters.of(2.75), 3300, hoodDatum + 20, 0.95);
    shotCalculator.putLUTEntry(Meters.of(3.00), 3450, hoodDatum + 20, 1.00);
    shotCalculator.putLUTEntry(Meters.of(3.20), 3550, hoodDatum + 20, 1.05);
    shotCalculator.putLUTEntry(Meters.of(3.50), 3800, hoodDatum + 20, 1.10);
    shotCalculator.putLUTEntry(Meters.of(3.80), 4000, hoodDatum + 20, 1.15);
    shotCalculator.putLUTEntry(Meters.of(4.00), 4075, hoodDatum + 20, 1.18);
    shotCalculator.putLUTEntry(Meters.of(4.20), 4150, hoodDatum + 20, 1.21);
    shotCalculator.putLUTEntry(Meters.of(4.50), 4300, hoodDatum + 20, 1.26);
    shotCalculator.putLUTEntry(Meters.of(5.00), 4600, hoodDatum + 20, 1.35);
    shotCalculator.putLUTEntry(Meters.of(5.25), 4750, hoodDatum + 20, 1.39);
    shotCalculator.putLUTEntry(Meters.of(5.50), 4600, hoodDatum + 25, 1.43);
    shotCalculator.putLUTEntry(Meters.of(5.60), 4750, hoodDatum + 25, 1.45);
    shotCalculator.putLUTEntry(Meters.of(6.00), 4900, hoodDatum + 25, 1.52);
    shotCalculator.putLUTEntry(Meters.of(6.50), 5200, hoodDatum + 25, 1.60);
    shotCalculator.putLUTEntry(Meters.of(6.75), 5450, hoodDatum + 25, 1.65);

    // passing
    passCalculator.putLUTEntry(Meters.of(3.00), 2900, hoodDatum + 25, 1.00);
    passCalculator.putLUTEntry(Meters.of(4.00), 3600, hoodDatum + 25, 1.20);
    passCalculator.putLUTEntry(Meters.of(5.00), 3850, hoodDatum + 25, 1.35);
    passCalculator.putLUTEntry(Meters.of(6.00), 4300, hoodDatum + 25, 1.50);
    passCalculator.putLUTEntry(Meters.of(7.00), 4800, hoodDatum + 30, 1.70);
    passCalculator.putLUTEntry(Meters.of(8.00), 5250, hoodDatum + 30, 1.85);
    passCalculator.putLUTEntry(Meters.of(10.0), 6000, hoodDatum + 30, 2.00);
    passCalculator.putLUTEntry(Meters.of(12.0), 6500, hoodDatum + 30, 2.50);
  }

  /** Odometry trust (0 - no trust, 1 - full trust) */
  @Logged(name = "OdometryTrust", importance = Importance.CRITICAL)
  private double odometryTrust = 0; // no trust initially

  /** Odometry frequency in Hz */
  @Logged(name = "OdometryFrequency", importance = Importance.DEBUG)
  private double odometryFrequency;

  /** Battery voltage */
  @Logged(name = "BatteryVoltage", importance = Importance.CRITICAL)
  private Voltage batteryVoltage = Volts.zero();

  /** Robot speed */
  @Logged(name = "RobotSpeed", importance = Importance.CRITICAL)
  private double robotSpeed = 0;

  /** Robot acceleration */
  @Logged(name = "RobotAcceleration", importance = Importance.CRITICAL)
  private double robotAcceleration = 0;

  /** Low battery condition */
  @Logged(name = "IsLowBattery", importance = Importance.CRITICAL)
  public boolean isLowBattery() {
    final var v = batteryVoltage.in(Volts);
    return v > 0 && v < kLowBatteryVoltage;
  }

  /** Fused robot pose */
  @Logged(name = "RobotPose", importance = Importance.CRITICAL)
  public Pose2d getRobotPose() {
    return drivetrain.getPose();
  }

  /** Vision robot pose */
  @Logged(name = "VisionPose", importance = Importance.CRITICAL)
  public Pose2d getVisionPose() {
    return visionPose;
  }

  /** Age (milliseconds) of the most recent vision pose */
  @Logged(name = "VisionPoseAge", importance = Importance.CRITICAL)
  public double getVisionPoseAge() {
    return visionPoseTimestamp > 0
        ? (RobotController.getTime() - visionPoseTimestamp) / 1000.0
        : -1;
  }

  /** Indicates that robot has received vision pose recently */
  @Logged(name = "HasRecentVisionPose", importance = Importance.CRITICAL)
  public boolean hasRecentVisionPose() {
    final var age = getVisionPoseAge();
    return age >= 0 && age <= kVisionCutoff / 1000.0;
  }

  /** Roll angle */
  @Logged(name = "Roll", importance = Importance.CRITICAL)
  public Angle getRoll() {
    return drivetrain.getRoll();
  }

  /** Pitch angle */
  @Logged(name = "Pitch", importance = Importance.CRITICAL)
  public Angle getPitch() {
    return drivetrain.getPitch();
  }

  /** Indicates that robot is level */
  @Logged(name = "IsLevel", importance = Importance.CRITICAL)
  public boolean isLevel() {
    return Math.abs(getRoll().in(Degrees)) < kLevelThreshold
        && Math.abs(getPitch().in(Degrees)) < kLevelThreshold;
  }

  /** Indicates that orientation lock is enabled */
  @Logged(name = "HasOrientationLock", importance = Importance.CRITICAL)
  public boolean hasOrientationLock() {
    return drivetrain.hasOrientationLock();
  }

  /** Indicates that odometry pose can be trusted */
  @Logged(name = "IsPoseTrusted", importance = Importance.CRITICAL)
  public boolean isPoseTrusted() {
    return odometryTrust >= kOdometryTrustThreshold;
  }

  /** Integrated swerve telemetry structure */
  @Logged(name = "SwerveTelemetry", importance = Importance.CRITICAL)
  public final SwerveTelemetry telemetry = new SwerveTelemetry();

  /** Indicator that auto win alliance information is missing */
  @Logged(name = "MissingAutoWinAlliance", importance = Importance.CRITICAL)
  public boolean missingAutoWinAlliance;

  /** Current game phase */
  @Logged(name = "Game/Phase", importance = Importance.CRITICAL)
  public Phase getGamePhase() {
    return phase;
  }

  /** Current game phase elapsed time */
  @Logged(name = "Game/PhaseElapsedTime", importance = Importance.CRITICAL)
  public Time getGamePhaseElapsedTime() {
    return phaseElapsedTime;
  }

  /** Current game phase remaining time */
  @Logged(name = "Game/PhaseRemainingTime", importance = Importance.CRITICAL)
  public Time getGamePhaseRemainingTime() {
    return phaseRemainingTime;
  }

  /** Hub active status */
  @Logged(name = "Game/HubActive", importance = Importance.CRITICAL)
  public boolean isHubActive() {
    return hubActive;
  }

  /** Robot's current field zone */
  @Logged(name = "Field/Zone", importance = Importance.CRITICAL)
  public Field.FieldZone getFieldZone() {
    return fieldZone;
  }

  /** Binds drivetrain. */
  public void bind(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.drivetrain.setVisionMeasurementStdDevs(kVisionStdDevs);
  }

  /** Binds external vision state supplier. */
  public void setVisionStateSupplier(Supplier<VisionState> value) {
    visionStateSupplier = value;
  }

  /** Binds external game phase supplier. */
  public void setGamePhaseSupplier(Supplier<PhaseInfo> value) {
    phaseInfoSupplier = value;
  }

  /** Resets odometry pose and restores trust. */
  private void resetOdometryPose(Pose2d pose) {
    odometryTrust = 1;
    drivetrain.resetPose(pose);
  }

  /** Determines turret angle to the target in degrees. */
  private double getTurretAngleTo(Translation2d target) {
    final var pose = getRobotPose();
    final var shooterXY = getShooterFieldPose(pose);

    // turret desired angle and zero in field frame of reference
    final var turretDesired = target.minus(shooterXY).getAngle();
    final var turretZero = pose.getRotation().plus(kTurretOffset);

    return turretDesired.minus(turretZero).getDegrees();
  }

  /** Returns the (X,Y) position of the shoooter in the field frame of reference. */
  public Translation2d getShooterFieldPose(Pose2d pose) {
    final var offsetXY = kShooterOffsetXY.rotateBy(pose.getRotation());
    return pose.getTranslation().plus(offsetXY);
  }

  /** Returns effective shooter parameters. */
  public Optional<Pair<Double, Double>> getShooterParameters() {
    return shotPossible
        ? Optional.of(Pair.of(shotAngle.lastValue(), shotVelocity.lastValue()))
        : Optional.empty();
  }

  /** Sets robot pose to a known pose and restores trust. */
  public void setRobotPose(Pose2d pose) {
    resetOdometryPose(pose);
  }

  /**
   * Runs logic at the beginning of the periodic cycle.
   *
   * <p>Use this to poll external inputs, e.g. operator or vision, before the command scheduler
   * runs.
   */
  public void enterPeriodic() {
    // synchronize all states to a common timestamp representing this update cycle
    final var timestamp = RobotController.getTime();

    batteryVoltage = RobotController.getMeasureBatteryVoltage();

    boolean isRobotStationary = false;
    boolean isRobotSlow = false;

    // poll odometry
    if (drivetrain != null) {
      final var driveState = drivetrain.getState();
      final var rotation = driveState.Pose.getRotation();
      final var robotSpeeds = driveState.Speeds;
      final var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, rotation);

      // average acceleration
      if (chassisSpeeds.size() > 0) {
        final var prevTimestampedSpeeds = chassisSpeeds.getHead();
        final var prevSpeeds =
            ChassisSpeeds.fromFieldRelativeSpeeds(prevTimestampedSpeeds.value(), rotation);

        // microseconds to seconds
        final var dt = (timestamp - prevTimestampedSpeeds.timestamp()) / 1e6;
        robotAx.calculate((robotSpeeds.vxMetersPerSecond - prevSpeeds.vxMetersPerSecond) / dt);
        robotAy.calculate((robotSpeeds.vyMetersPerSecond - prevSpeeds.vyMetersPerSecond) / dt);
        robotAw.calculate(
            (robotSpeeds.omegaRadiansPerSecond - prevSpeeds.omegaRadiansPerSecond) / dt);
        robotAcceleration = Math.hypot(robotAx.lastValue(), robotAy.lastValue());
      }

      chassisSpeeds.enqueue(timestamp, fieldSpeeds);

      // reduce trust in odometry if slippage is detected
      final var maxSlipRatio =
          Arrays.stream(moduleSlipRatiosArray).map(Math::abs).max().getAsDouble();
      if (maxSlipRatio > kRobotSlipThreshold) {
        odometryTrust = Math.max(0, odometryTrust - kOdometryTrustPenalty);
      }

      isRobotStationary =
          MathUtil.isNear(0, robotSpeeds.vxMetersPerSecond, kStationarySpeedTolerance)
              && MathUtil.isNear(0, robotSpeeds.vyMetersPerSecond, kStationarySpeedTolerance)
              && MathUtil.isNear(0, robotSpeeds.omegaRadiansPerSecond, kStationaryOmegaTolerance);

      isRobotSlow =
          MathUtil.isNear(0, robotSpeeds.vxMetersPerSecond, kSlowSpeedTolerance)
              && MathUtil.isNear(0, robotSpeeds.vyMetersPerSecond, kSlowSpeedTolerance)
              && MathUtil.isNear(0, robotSpeeds.omegaRadiansPerSecond, kSlowOmegaTolerance);
    }

    if (kVisionAssist.get()) {
      // poll vision state
      if (visionStateSupplier != null) {
        final var visionState = visionStateSupplier.get();

        // check if we have received a newer vision pose
        final var newVisionPoseTimestamp = visionState.pose.timestamp();
        if (newVisionPoseTimestamp > visionPoseTimestamp) {
          visionPose = visionState.pose.value();

          // special handling of the very first vision update:
          // if the robot is stationary set odometry pose to the vision pose
          if (visionPoseTimestamp <= 0 && isRobotStationary) {
            resetOdometryPose(visionPose);
          }

          visionPoseTimestamp = newVisionPoseTimestamp;
        }
      }

      // fuse robot pose with vision
      if (visionPoseTimestamp > 0 && visionPoseTimestamp > timestamp - kVisionCutoff) {
        // reset robot pose to vision pose when moving slowly and odometry is not trusted
        if (isRobotSlow && odometryTrust < kOdometryTrustThreshold) {
          resetOdometryPose(visionPose);
        } else {
          drivetrain.addVisionMeasurement(
              visionPose, Utils.fpgaToCurrentTime(visionPoseTimestamp / 1000000.0));
        }
      }
    }

    // sync game phase info
    final var phaseInfo = phaseInfoSupplier.get();
    phase = phaseInfo.phase();
    phaseElapsedTime = Seconds.of(phaseInfo.elapsedTime());
    phaseRemainingTime = Seconds.of(phaseInfo.remainingTime());
    hubActive = phaseInfo.active();

    final var pose = getRobotPose();
    final var poseXY = pose.getTranslation();
    final var speeds = drivetrain.getFieldSpeeds();

    // update robot's current zone
    fieldZone = Field.getZone(poseXY);

    // determine effective turret angle and shot parameters
    if (Field.isInPassingZone(poseXY)) {
      shotParameters = passCalculator.calculate(pose, speeds, getPassingTarget());

      // always allow passing
      shotPossible = true;
    } else {
      // use predicted pose to compensate for the feeder mechanism delay
      shotParameters =
          shotCalculator.calculate(
              getPredictedPose(kMechanismDelay), speeds, Field.transform(Field.kHubCenter));

      // prepare the shot but do not shoot unless actually within the shooting zone
      shotPossible =
          Field.isInShootingZone(poseXY)
              && shotParameters.quality() >= 0.5
              && shotParameters.distance().gte(kMinScoreDistance)
              && shotParameters.distance().lte(kMaxScoreDistance)
              && robotSpeed <= kMaxScoreSpeed
              && !isShotSuppressed();
    }

    shotVelocity.calculate(shotParameters.shooterVelocity());
    shotAngle.calculate(shotParameters.shooterAngle());
    turretAngle.calculate(getTurretAngleTo(shotParameters.target()));
  }

  /**
   * Runs logic at the end of the periodic cycle.
   *
   * <p>Use this to synchronize the state with external modules, after the command scheduler runs.
   */
  public void exitPeriodic() {}

  /** Runs logic at the beginning of the autonomous cycle. */
  public void enterAutonomous() {}

  /** Runs logic at the beginning of the teleop cycle. */
  public void enterTeleop() {}

  /** Returns observed chassis speeds at the specified timestamp. */
  public Optional<ChassisSpeeds> getSpeedsAt(long timestamp) {
    return chassisSpeeds.findAt(timestamp);
  }

  /** Returns observed pose at the specified timestamp. */
  public Optional<Pose2d> getOdometryPoseAt(long timestamp) {
    return drivetrain != null ? drivetrain.getPoseAt(timestamp) : Optional.empty();
  }

  /** Returns predicted pose at the specified time into the future. */
  public Pose2d getPredictedPose(Time t) {
    final var rv = drivetrain.getRobotSpeeds();
    final var dt = t.in(Seconds);
    final var dt2 = 0.5 * dt * dt;
    final var twist =
        new Twist2d(
            rv.vxMetersPerSecond * dt + robotAx.lastValue() * dt2,
            rv.vyMetersPerSecond * dt + robotAy.lastValue() * dt2,
            rv.omegaRadiansPerSecond * dt + robotAw.lastValue() * dt2);

    return getRobotPose().exp(twist);
  }

  private final double[] poseArray = new double[3];
  private final double[] moduleStatesArray = new double[8];
  private final double[] moduleTargetsArray = new double[8];
  private final double[] moduleSlipRatiosArray = new double[4];

  /** Invoked by CTRE odometry thread. */
  public void updateSwerveTelemetry(SwerveDriveState state) {
    telemetry.rotation = state.Pose.getRotation();
    telemetry.currentSpeeds = state.Speeds;
    telemetry.currentStates = state.ModuleStates;
    telemetry.desiredStates = state.ModuleTargets;
    odometryFrequency = 1.0 / state.OdometryPeriod;
    robotSpeed = Math.hypot(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);

    computeSlipRatios(state, moduleSlipRatiosArray);

    // signal logger
    if (Constants.Settings.ctreLoggingEnabled) {
      poseArray[0] = state.Pose.getX();
      poseArray[1] = state.Pose.getY();
      poseArray[2] = state.Pose.getRotation().getDegrees();

      for (var i = 0; i < state.ModuleStates.length; ++i) {
        moduleStatesArray[i * 2 + 0] = state.ModuleStates[i].angle.getRadians();
        moduleStatesArray[i * 2 + 1] = state.ModuleStates[i].speedMetersPerSecond;
        moduleTargetsArray[i * 2 + 0] = state.ModuleTargets[i].angle.getRadians();
        moduleTargetsArray[i * 2 + 1] = state.ModuleTargets[i].speedMetersPerSecond;
      }

      SignalLogger.writeDoubleArray("DriveState/Pose", poseArray);
      SignalLogger.writeDoubleArray("DriveState/ModuleStates", moduleStatesArray);
      SignalLogger.writeDoubleArray("DriveState/ModuleTargets", moduleTargetsArray);
      SignalLogger.writeDoubleArray("DriveState/ModuleSlipRatios", moduleSlipRatiosArray);
      SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");
    }
  }

  /** Computes slip ratios for each module and populates the array. */
  private static void computeSlipRatios(SwerveDriveState state, double[] slipRatios) {
    // detect slippage by comparing translation velocities of the wheels with IMU velocity
    final var robotVelocity =
        new Translation2d(state.Speeds.vxMetersPerSecond, state.Speeds.vyMetersPerSecond);

    final var robotVelocityNorm = robotVelocity.getNorm();

    for (var i = 0; i < state.ModuleStates.length; ++i) {
      final var moduleVelocity =
          new Translation2d(
              state.ModuleStates[i].speedMetersPerSecond, state.ModuleStates[i].angle);

      final var moduleTranslationVelocity =
          new Translation2d(
              moduleVelocity.getX()
                  + state.Speeds.omegaRadiansPerSecond * TunerConstants.modulePositions[i].getY(),
              moduleVelocity.getY()
                  - state.Speeds.omegaRadiansPerSecond * TunerConstants.modulePositions[i].getX());

      // robot can be stationary, but can still slip if we are trying to push against an obstacle
      if (MathUtil.isNear(0, robotVelocityNorm, kStationaryVelocityTolerance)) {
        slipRatios[i] =
            MathUtil.isNear(0, moduleTranslationVelocity.getNorm(), kStationaryVelocityTolerance)
                ? 0
                : 1;
      } else {
        // technically we should compute projections on the robot IMU velocity vector
        // but all these vectors should be colinear unless the robot is falling apart
        slipRatios[i] =
            (moduleTranslationVelocity.getNorm() - robotVelocityNorm) / robotVelocityNorm;
      }
    }
  }

  /** Sets passing target. */
  private void setPassingTarget(PassingPoint targetPoint) {
    passingPoint = targetPoint;
  }

  /** Sets passing target to the default point. */
  public Command setPassingToDefault() {
    return Commands.runOnce(() -> setPassingTarget(PassingPoint.kDefault));
  }

  /** Sets passing target to the auto trench zone. */
  public Command setPassingToTrenchAuto() {
    return Commands.runOnce(() -> setPassingTarget(PassingPoint.kTrenchAuto));
  }

  /** Sets passing target to the left trench zone. */
  public Command setPassingToTrenchLeft() {
    return Commands.runOnce(() -> setPassingTarget(PassingPoint.kTrenchLeft));
  }

  /** Sets passing target to the right trench zone. */
  public Command setPassingToTrenchRight() {
    return Commands.runOnce(() -> setPassingTarget(PassingPoint.kTrenchRight));
  }

  /** Returns appropriate passing target based on the selection and robot pose. */
  @Logged(name = "PassingTarget", importance = Importance.CRITICAL)
  public Translation2d getPassingTarget() {
    return getPassingTarget(passingPoint);
  }

  /** Returns appropriate passing point based on the selection and robot pose. */
  public Translation2d getPassingTarget(PassingPoint targetPoint) {
    final var pose = getRobotPose();

    // determine what Y side of the field we are on
    var originY = 0.0;
    var signY = 1.0;

    if (pose.getY() > Field.kFieldHalfWidth) {
      originY = Field.kFieldWidth;
      signY = -1.0;
    }

    switch (targetPoint) {
      case kTrenchRight:
        return new Translation2d(
            Field.transformX(Field.kStartingLineX), Field.transformY(Field.kTrenchWidth * 0.75));

      case kTrenchLeft:
        return new Translation2d(
            Field.transformX(Field.kStartingLineX),
            Field.transformY(Field.kFieldWidth - Field.kTrenchWidth * 0.75));

      case kTrenchAuto:
        return new Translation2d(
            Field.transformX(Field.kStartingLineX), originY + signY * Field.kTrenchWidth * 0.75);

      default:
        return new Translation2d(
            Field.transformX(Field.kDefaultPassingPoint.getX()),
            originY + signY * Field.kDefaultPassingPoint.getY());
    }
  }

  /** Determines whether robot is away from the tower zone. */
  public boolean isAwayFromTower() {
    return Field.isAwayFromTower(getRobotPose());
  }

  private static final double kTrenchZonePredictiveTime = 0.25; // seconds

  /** Determines whether shooter position is within trench zone. */
  public boolean isShooterWithinTrench() {
    return Field.isInTrenchZone(
        getShooterFieldPose(getRobotPose()),
        drivetrain.getFieldSpeeds(),
        kTrenchZonePredictiveTime);
  }

  @Logged(name = "Tuning/ActualShotDistance", importance = Importance.CRITICAL)
  public Distance getActualShotDistance() {
    final var target = Field.transform(Field.kHubCenter);
    return Meters.of(getShooterFieldPose(getRobotPose()).minus(target).getNorm());
  }

  @Logged(name = "Tuning/ActualPassDistance", importance = Importance.DEBUG)
  public Distance getActualPasstDistance() {
    final var target = getPassingTarget(passingPoint);
    return Meters.of(getShooterFieldPose(getRobotPose()).minus(target).getNorm());
  }

  private static final Tunable.DoubleValue tuningShotDistance =
      Tunable.value("Tuning/RequiredShotDistance", 3);

  @Logged(name = "Tuning/ShotPose", importance = Importance.DEBUG)
  public Pose2d getTuningShotPose() {
    final var target = Field.transform(Field.kHubCenter);
    final var corner = Field.transform(new Translation2d(0, Field.kFieldWidth));

    final var angle = target.minus(corner).getAngle();
    final var position = new Translation2d(tuningShotDistance.get(), 0).rotateBy(angle);

    final var robotRotation = angle.rotateBy(Rotation2d.k180deg);
    final var offsetXY = kShooterOffsetXY.rotateBy(robotRotation);

    return new Pose2d(target.minus(position).minus(offsetXY), robotRotation);
  }
}
