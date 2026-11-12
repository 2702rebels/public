package frc.robot;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.PubSubOption;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.MatchType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.lib.ReefState;
import frc.robot.lib.TimestampedCircularBuffer;
import frc.robot.lib.TimestampedTranslation2d;
import frc.robot.lib.Tunable;
import frc.robot.lib.VisionState;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import java.util.Arrays;
import java.util.BitSet;
import java.util.Optional;
import java.util.function.Supplier;

public class RobotState {
  private static RobotState instance;

  public static RobotState getInstance() {
    if (instance == null) instance = new RobotState();
    return instance;
  }

  private static final int kCoralsPerLevelForRP = 5;
  private static final int kCoralsPerLevel = 12;

  private static final long kVisionCutoff = 60 * 1000; // microseconds

  private static final double kStationarySpeedTolerance = 0.05; // 5 cm/sec
  private static final double kStationaryOmegaTolerance = Units.degreesToRadians(5); // 5 deg/sec
  private static final double kSlowSpeedTolerance = 1; // 1.0 m/sec
  private static final double kSlowOmegaTolerance = Units.degreesToRadians(60); // 60 deg/sec
  private static final double kRobotSlipThreshold = 0.3;
  private static final double kOdometryTrustThreshold = 0.8;
  private static final double kOdometryTrustPenalty = 0.05;

  // std.dev for vision: we do trust vision so set these relatively low
  private static final Matrix<N3, N1> kVisionStdDevs =
      VecBuilder.fill(
          Units.inchesToMeters(0.5), Units.inchesToMeters(0.5), Units.degreesToRadians(5));

  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();

  // timestamped log of recently measured (within last second) chassis speeds
  private final TimestampedCircularBuffer<ChassisSpeeds> chassisSpeeds =
      new TimestampedCircularBuffer<>(50);

  // bound drivetrain
  private CommandSwerveDrivetrain drivetrain;

  // external vision state supplier
  private Supplier<VisionState> visionStateSupplier;

  // external object detection supplier
  private Supplier<TimestampedTranslation2d> detectedObjectSupplier;

  private long visionPoseTimestamp = -1;
  private long coralPoseTimestamp = -1;

  private Pose2d visionPose = Pose2d.kZero;

  private Optional<Pose2d> coralPose = Optional.empty();
  private Optional<Pose2d> targetPose = Optional.empty();
  private Optional<Pose2d> scoringPose = Optional.empty();
  private int scoringTarget = -1;

  // tracks number of automatic scoring retries when scoring L4
  private int autoRetryCountL4 = 0;

  // tracks steps in multi-coral auto sequences
  private int autoScoringStep = 0;

  // indicator of automatic scoring mode
  private boolean auto;

  /*
   * ReefState to use with Reef widgets
   *
   * We are "mimicking" sendable data structure here (by including .type field)
   * since the widget requires two separate topics - one for sending operator's input,
   * and one for representing the robot's perspective.
   *
   * The robot code is responsible for merging the operator's and robot's perspective
   * into a consistent state. This approach avoids the problem of "missing updates"
   * that are possible if both sides were to modify the same data structure concurrently.
   */
  private final NetworkTable reefStateTable = nt.getTable("Robot/State/Reef");
  private final StringPublisher reefStateTypePublisher =
      reefStateTable.getStringTopic(".type").publish();
  private final StructPublisher<ReefState> reefStateRobotPublisher;
  private final StructSubscriber<ReefState> reefStateOperatorSubscriber;

  private final SendableChooser<ScoringStrategy> scoringStrategyChooser = new SendableChooser<>();

  private RobotState() {
    reefState = new ReefState();

    scoringStrategyChooser.setDefaultOption("Auto", ScoringStrategy.kAuto);
    scoringStrategyChooser.addOption("Ranking Point", ScoringStrategy.kRankingPoint);
    scoringStrategyChooser.addOption("Score", ScoringStrategy.kScore);

    reefStateRobotPublisher = reefStateTable.getStructTopic("robot", ReefState.struct).publish();
    reefStateOperatorSubscriber =
        reefStateTable
            .getStructTopic("operator", ReefState.struct)
            .subscribe(new ReefState(), PubSubOption.sendAll(false));

    SmartDashboard.putData("Scoring Strategy", scoringStrategyChooser);

    reefStateTypePublisher.setDefault("ReefState");
    reefStateRobotPublisher.setDefault(new ReefState());
  }

  /** Reef state (robot side) */
  public final ReefState reefState;

  public static enum Intent {
    kNone,

    /** Hover intake to allow trapped game pieces to escape */
    kHover,

    /** Intake coral form the floor */
    kCoralFloorIntake,

    /** Intake coral from the loading station */
    kCoralStationIntake,

    /** Eject coral */
    kCoralEject,

    /** Score coral at L1 */
    kCoralScoreL1,

    /** Score coral at L2 */
    kCoralScoreL2,

    /** Score coral at L3 */
    kCoralScoreL3,

    /** Score coral at L4 */
    kCoralScoreL4,

    /** Score coral at L2, L3, L4 per reef scoring strategy */
    kCoralScoreAuto,

    /** Score coral at L4 (final motion) */
    kCoralScoreL4Final,

    /** Score coral at L4 (retry motion) */
    kCoralScoreL4Retry,

    /** Intake algae (L2 low) */
    kAlgaeIntakeL2,

    /** Intake algae (L3 high) */
    kAlgaeIntakeL3,

    /** Intake algae ground */
    kAlgaeIntakeGround,

    /** Score algae into the net */
    kAlgaeScoreNet,

    /** Eject algae into the processor */
    kAlgaeEject,

    /** Prepare for climbing (deploy climber) */
    kClimbReady,

    /** Engage climber */
    kClimbEngage,

    /** Ready state when holding onto coral */
    kCoralReady,
  }

  public static enum ScoringStrategy {
    /** Automatic based on the match type (qualification / playoff) */
    kAuto,

    /** Meet RP requirement */
    kRankingPoint,

    /** Maximize total score */
    kScore,
  }

  /** Scoring strategy for the reef */
  @Logged(name = "ScoringStrategy", importance = Importance.INFO)
  public ScoringStrategy scoringStrategy = ScoringStrategy.kAuto;

  /** Robot intent */
  @Logged(name = "Intent", importance = Importance.INFO)
  public Intent intent = Intent.kNone;

  /** Odometry trust (0 - no trust, 1 - full trust) */
  @Logged(name = "OdometryTrust", importance = Importance.INFO)
  private double odometryTrust = 0; // no trust initially

  /** Fused robot pose */
  @Logged(name = "RobotPose", importance = Importance.INFO)
  public Pose2d getRobotPose() {
    return drivetrain.getPose();
  }

  /** Vision robot pose */
  @Logged(name = "VisionPose", importance = Importance.INFO)
  public Pose2d getVisionPose() {
    return visionPose;
  }

  /** Age (milliseconds) of the most recent vision pose */
  @Logged(name = "VisionPoseAgeMS", importance = Importance.DEBUG)
  public double getVisionPoseAge() {
    return visionPoseTimestamp > 0
        ? (RobotController.getFPGATime() - visionPoseTimestamp) / 1000.0
        : -1;
  }

  /** Detected coral pose (debugging only) */
  @Logged(name = "CoralPose", importance = Importance.INFO)
  public Pose2d getCoralPoseDebug() {
    return coralPose.isPresent() ? coralPose.get() : Pose2d.kZero;
  }

  /** Age (milliseconds) of the most recent coral pose */
  @Logged(name = "CoralPoseAgeMS", importance = Importance.DEBUG)
  public double getCoralPoseAge() {
    return coralPoseTimestamp > 0
        ? (RobotController.getFPGATime() - coralPoseTimestamp) / 1000.0
        : -1;
  }

  /** Scoring pose (debugging only) */
  @Logged(name = "ScoringPose", importance = Importance.DEBUG)
  public Pose2d getScoringPoseDebug() {
    return scoringPose.isPresent() ? scoringPose.get() : Pose2d.kZero;
  }

  /** Roll angle */
  @Logged(name = "Roll", importance = Importance.INFO)
  public Angle getRoll() {
    return drivetrain.getRoll();
  }

  /** Pitch angle */
  @Logged(name = "Pitch", importance = Importance.INFO)
  public Angle getPitch() {
    return drivetrain.getPitch();
  }

  // spotless:off
  @Logged(name = "Idle", importance = Importance.INFO)
  public final Trigger idle = new Trigger(() -> intent == Intent.kNone);

  @Logged(name = "CoralReady", importance = Importance.INFO)
  public final Trigger coralReady = new Trigger(() -> intent == Intent.kCoralReady);

  @Logged(name = "ShouldHover", importance = Importance.INFO)
  public final Trigger shouldHover = new Trigger(() -> intent == Intent.kHover);

  @Logged(name = "ShouldIntakeCoralFloor", importance = Importance.INFO)
  public final Trigger shouldIntakeCoralFloor = new Trigger(() -> intent == Intent.kCoralFloorIntake);

  @Logged(name = "ShouldIntakeCoralStation", importance = Importance.INFO)
  public final Trigger shouldIntakeCoralStation = new Trigger(() -> intent == Intent.kCoralStationIntake);

  @Logged(name = "ShouldIntakeCoral", importance = Importance.INFO)
  public final Trigger shouldIntakeCoral = shouldIntakeCoralFloor.or(shouldIntakeCoralStation);

  @Logged(name = "ShouldEjectCoral", importance = Importance.INFO)
  public final Trigger shouldEjectCoral = new Trigger(() -> intent == Intent.kCoralEject);

  @Logged(name = "ShouldScoreCoralL1", importance = Importance.INFO)
  public final Trigger shouldScoreCoralL1 = new Trigger(() -> intent == Intent.kCoralScoreL1);

  @Logged(name = "ShouldScoreCoralL2", importance = Importance.INFO)
  public final Trigger shouldScoreCoralL2 = new Trigger(() -> intent == Intent.kCoralScoreL2);

  @Logged(name = "ShouldScoreCoralL3", importance = Importance.INFO)
  public final Trigger shouldScoreCoralL3 = new Trigger(() -> intent == Intent.kCoralScoreL3);

  @Logged(name = "ShouldScoreCoralL4", importance = Importance.INFO)
  public final Trigger shouldScoreCoralL4 = new Trigger(() -> intent == Intent.kCoralScoreL4);

  @Logged(name = "ShouldScoreCoralL4Final", importance = Importance.INFO)
  public final Trigger shouldScoreCoralL4Final = new Trigger(() -> intent == Intent.kCoralScoreL4Final);

  @Logged(name = "ShouldScoreCoralL4Retry", importance = Importance.INFO)
  public final Trigger shouldScoreCoralL4Retry = new Trigger(() -> intent == Intent.kCoralScoreL4Retry);

  @Logged(name = "ShouldIntakeAlgaeL2", importance = Importance.INFO)
  public final Trigger shouldIntakeAlgaeL2 = new Trigger(() -> intent == Intent.kAlgaeIntakeL2);

  @Logged(name = "ShouldIntakeAlgaeL3", importance = Importance.INFO)
  public final Trigger shouldIntakeAlgaeL3 = new Trigger(() -> intent == Intent.kAlgaeIntakeL3);

  @Logged(name = "ShouldIntakeAlgaeGround", importance = Importance.INFO)
  public final Trigger shouldIntakeAlgaeGround = new Trigger(() -> intent == Intent.kAlgaeIntakeGround);

  @Logged(name = "ShouldIntakeAlgae", importance = Importance.INFO)
  public final Trigger shouldIntakeAlgae = shouldIntakeAlgaeL2.or(shouldIntakeAlgaeL3).or(shouldIntakeAlgaeGround);

  @Logged(name = "ShouldEjectAlgae", importance = Importance.INFO)
  public final Trigger shouldEjectAlgae = new Trigger(() -> intent == Intent.kAlgaeEject);

  @Logged(name = "ShouldScoreAlgaeNet", importance = Importance.INFO)
  public final Trigger shouldScoreAlgaeNet = new Trigger(() -> intent == Intent.kAlgaeScoreNet);

  @Logged(name = "ShouldClimbReady", importance = Importance.INFO)
  public final Trigger shouldClimbReady = new Trigger(() -> intent == Intent.kClimbReady);

  @Logged(name = "ShouldClimbEngage", importance = Importance.INFO)
  public final Trigger shouldClimbEngage = new Trigger(() -> intent == Intent.kClimbEngage);

  @Logged(name = "Auto", importance = Importance.INFO)
  public final Trigger isAuto = new Trigger(() -> auto);

  @Logged(name = "AutoScore", importance = Importance.INFO)
  public final Trigger isAutoScore = new Trigger(scoreAssist::get);

  @Logged(name = "ShouldScoreL4Eject", importance = Importance.INFO)
  public final Trigger shouldScoreL4Eject = new Trigger(scoreL4Eject::get);

  @Logged(name = "IsPoseTrusted", importance = Importance.INFO)
  public final Trigger isPoseTrusted = new Trigger(() -> odometryTrust >= kOdometryTrustThreshold);

  @Logged(name = "HasScoringPose", importance = Importance.INFO)
  public final Trigger hasScoringPose = new Trigger(() -> scoringPose.isPresent());

  @Logged(name = "AtScoringPose", importance = Importance.INFO)
  public final Trigger atScoringPose = new Trigger(this::isAtScoringPose);
  // spotless:on

  /** Binds drivetrain. */
  public void bind(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    this.drivetrain.setVisionMeasurementStdDevs(kVisionStdDevs);
  }

  /** Binds external vision state supplier. */
  public void setVisionStateSupplier(Supplier<VisionState> value) {
    visionStateSupplier = value;
  }

  /** Binds external object detection supplier. */
  public void setDetectedObjectSupplier(Supplier<TimestampedTranslation2d> value) {
    detectedObjectSupplier = value;
  }

  /** Gets the scoring level for automatic scoring (1..4) */
  @Logged(name = "ScoringLevel", importance = Importance.INFO)
  public int getScoringLevel() {
    return reefState.level > 0
        ? reefState.level
        : reefState.target > 0 ? (reefState.target / 12) + 1 : -1;
  }

  /** Commands intent of the robot. */
  public Command setIntent(Intent intent) {
    return Commands.runOnce(
            () -> {
              if (intent == Intent.kCoralScoreAuto) {
                // must have scoring level selected
                final var scoringLevel = getScoringLevel();
                if (scoringLevel <= 0) {
                  return;
                }

                // if vision assist is enabled and we trust current pose
                // then try to acquire scoring pose based on the closest
                // un-scored position, switch to auto mode if successful
                if (visionAssist.get() && isPoseTrusted.getAsBoolean()) {
                  final var scoringPoseAndTarget =
                      getClosestScoringPose(getRobotPose(), scoringLevel);

                  if (scoringPoseAndTarget.isPresent()) {
                    auto = true;
                    scoringPose = Optional.of(scoringPoseAndTarget.get().getFirst());
                    scoringTarget = scoringPoseAndTarget.get().getSecond();
                  }
                }

                switch (scoringLevel) {
                  case 1:
                    this.intent = Intent.kCoralScoreL1;
                    break;
                  case 2:
                    this.intent = Intent.kCoralScoreL2;
                    break;
                  case 3:
                    this.intent = Intent.kCoralScoreL3;
                    break;
                  case 4:
                    this.intent = Intent.kCoralScoreL4;
                    autoRetryCountL4 = 0;
                    break;
                }
              } else {
                // reset scoring pose when intent is removed
                scoringPose = Optional.empty();

                // do not reset target/auto if still within L4 sequence
                if (intent != Intent.kCoralScoreL4Final && intent != Intent.kCoralScoreL4Retry) {
                  scoringTarget = -1;
                  auto = false;
                }

                if (intent == Intent.kCoralScoreL4Retry) {
                  autoRetryCountL4++;
                }

                this.intent = intent;
              }
            })
        .withName("SetIntent");
  }

  /**
   * Commands scoring at L4 in auto mode, aborts if pose is not trusted.
   *
   * @param target Index of the reef scoring pose (see GameConstants) or -1 to use closest pose.
   */
  public Command scoreAutoL4(int target) {
    return Commands.runOnce(
        () -> {
          if (!isPoseTrusted.getAsBoolean() || target >= 12) {
            return;
          }

          if (target < 0) {
            final var scoringPoseAndTarget = getClosestScoringPose(getRobotPose(), 4);
            if (scoringPoseAndTarget.isEmpty()) {
              return;
            }

            scoringTarget = scoringPoseAndTarget.get().getSecond();
            scoringPose = Optional.of(scoringPoseAndTarget.get().getFirst());
          } else {
            scoringTarget = 48 + target;
            scoringPose =
                Optional.of(
                    FieldConstants.fieldPose.apply(GameConstants.reefScoringPosesL4[target]));
          }

          intent = Intent.kCoralScoreL4;
          autoRetryCountL4 = 0;
          auto = true;
          autoScoringStep++;
        });
  }

  /** Gets auto-scoring step number. */
  public int getAutoScoringStep() {
    return autoScoringStep;
  }

  /** Sets target pose to drive to. */
  public void setTargetPose(Pose2d pose) {
    targetPose = Optional.of(pose);
  }

  /** Gets target pose to drive to. */
  public Optional<Pose2d> getTargetPose() {
    return targetPose;
  }

  /** Gets scoring pose to drive to. */
  public Optional<Pose2d> getScoringPose() {
    return scoringPose;
  }

  /** Gets detected coral pose. */
  public Optional<Pose2d> getCoralPose() {
    return coralPose;
  }

  /** Determines whether robot is at the scoring pose. */
  public boolean isAtScoringPose() {
    return scoringPose.isPresent() && drivetrain.isAtPose(scoringPose.get());
  }

  /** Command to reset scoring pose. */
  public Command resetScoringPose() {
    return new InstantCommand(() -> scoringPose = Optional.empty());
  }

  /** Command to reset auto mode back to manual. */
  public Command resetAuto() {
    return new InstantCommand(() -> auto = false);
  }

  /** Command to mark scoring target as scored */
  public Command markScoringTarget() {
    return new InstantCommand(
        () -> {
          if (scoringTarget > 0 && auto && targetAssist.get()) {
            reefState.markTargetScored(scoringTarget);
            scoringTarget = -1;
          }
        });
  }

  /** Synchronizes operator's input with the robot's perspective. */
  private void pollReefState() {
    final var values = reefStateOperatorSubscriber.readQueueValues();
    if (values.length == 0) return;

    for (var i = 0; i < values.length; ++i) {
      final var value = values[i];

      // set the robot's perspective to the operator's input
      reefState.algae = (BitSet) value.algae.clone();
      reefState.coralsL1 = value.coralsL1;
      reefState.coralsL2 = (BitSet) value.coralsL2.clone();
      reefState.coralsL3 = (BitSet) value.coralsL3.clone();
      reefState.coralsL4 = (BitSet) value.coralsL4.clone();
      reefState.target = value.target;
      reefState.locked = value.locked;
      reefState.level = value.level;
    }

    // if target position is marked as scored, remove the target and the lock
    if (reefState.isTargetScored()) {
      reefState.target = -1;
      reefState.locked = false;
    }
  }

  // maintains most recent module slip ratios (to avoid allocations)
  private final double[] moduleSlipRatios = new double[4];

  /** Resets odometry pose and restores trust. */
  private void resetOdometryPose(Pose2d pose) {
    odometryTrust = 1;
    drivetrain.resetPose(pose);
  }

  /** Sets robot pose to a known pose and restores trust. */
  public void setRobotPose(Pose2d pose) {
    resetOdometryPose(pose);
  }

  /** Determines whether L4 automatic retry can proceed. */
  public boolean canAutoRetryL4() {
    return autoRetryCountL4 <= 2; // 2 attempts
  }

  private static final Rotation2d kAutoRetryL4RotationAttempt1 = Rotation2d.fromDegrees(2);
  private static final Rotation2d kAutoRetryL4RotationAttempt2 = Rotation2d.fromDegrees(-4);

  /** Returns requested robot pose on L4 automatic retry. */
  public Optional<Pose2d> getAutoRetryL4Pose() {
    switch (autoRetryCountL4) {
      case 1:
        return Optional.of(getRobotPose().rotateBy(kAutoRetryL4RotationAttempt1));
      case 2:
        return Optional.of(getRobotPose().rotateBy(kAutoRetryL4RotationAttempt2));
      default:
        return Optional.empty();
    }
  }

  /**
   * Runs logic at the beginning of the periodic cycle.
   *
   * <p>Use this to poll external inputs, e.g. operator or vision, before the command scheduler
   * runs.
   */
  public void enterPeriodic() {
    pollReefState();

    // synchronize all states to a common timestamp representing this update cycle
    final var timestamp = RobotController.getFPGATime();

    boolean isRobotStationary = false;
    boolean isRobotSlow = false;

    // poll odometry
    if (drivetrain != null) {
      final var driveState = drivetrain.getState();
      final var rotation = driveState.Pose.getRotation();
      final var robotSpeeds = driveState.Speeds;
      final var fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotSpeeds, rotation);
      chassisSpeeds.enqueue(timestamp, fieldSpeeds);

      Telemetry.computeSlipRatios(driveState, moduleSlipRatios);

      // reduce trust in odometry if slippage is detected
      final var maxSlipRatio = Arrays.stream(moduleSlipRatios).map(Math::abs).max().getAsDouble();
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

    // poll detected objects
    if (detectedObjectSupplier != null) {
      final var detectedObject = detectedObjectSupplier.get();
      final var newDetectedObjectTimestamp = detectedObject.timestamp();
      if (newDetectedObjectTimestamp > coralPoseTimestamp) {
        final var vectorToCoral = detectedObject.value();

        // determine pose of the coral based on the robot pose and heading
        final var maybeRobotPoseAtDetection = getOdometryPoseAt(newDetectedObjectTimestamp);
        if (maybeRobotPoseAtDetection.isPresent()) {
          final var robotPoseAtDetection = maybeRobotPoseAtDetection.get();
          coralPoseTimestamp = newDetectedObjectTimestamp;
          coralPose =
              Optional.of(
                  new Pose2d(
                      robotPoseAtDetection
                          .getTranslation()
                          .plus(vectorToCoral.rotateBy(robotPoseAtDetection.getRotation())),
                      Rotation2d.kZero));
        }
      }
    }

    scoringStrategy = scoringStrategyChooser.getSelected();

    // if the reef target is not locked by the operator use the scoring strategy
    // to automatically pick the most appropriate target based on the current state
    if (!reefState.locked && DriverStation.isTeleopEnabled()) {
      final var effectiveScoringStrategy =
          scoringStrategy == ScoringStrategy.kAuto
              ? DriverStation.getMatchType() == MatchType.Elimination
                  ? ScoringStrategy.kScore
                  : ScoringStrategy.kRankingPoint
              : scoringStrategy;

      // fill levels starting from the top (zero-based indexing here, i.e. L1 = 0, L2 = 1, etc.)
      var targetLevel = -1;

      if (effectiveScoringStrategy == ScoringStrategy.kRankingPoint) {
        if (reefState.coralsL4.cardinality() < kCoralsPerLevelForRP) {
          targetLevel = 3;
        } else if (reefState.coralsL3.cardinality() < kCoralsPerLevelForRP) {
          targetLevel = 2;
        } else if (reefState.coralsL2.cardinality() < kCoralsPerLevelForRP) {
          targetLevel = 1;
        }
      }

      // if target level is not set with RP strategy above, it means that we think
      // RP has been achieved, so we should focus on scoring
      if (effectiveScoringStrategy == ScoringStrategy.kScore || targetLevel < 0) {
        if (reefState.coralsL4.cardinality() < kCoralsPerLevel) {
          targetLevel = 3;
        } else if (reefState.coralsL3.cardinality() < kCoralsPerLevel) {
          targetLevel = 2;
        } else if (reefState.coralsL2.cardinality() < kCoralsPerLevel) {
          targetLevel = 1;
        } else {
          targetLevel = 0;
        }
      }

      // scoring at L1 is manual, nothing to do here
      if (targetLevel == 0) {
        reefState.target = -1;
      } else {
        // TODO: find the closest scoring position based on the robot location, other factors
        final var corals = reefState.getCorals(targetLevel);
        reefState.target = (byte) (targetLevel * 12 + corals.nextClearBit(0));
      }
    }
  }

  /** Gets the closest scoring pose based on the closest reef faces. */
  private Optional<Pair<Pose2d, Integer>> getClosestScoringPose(
      Pose2d robotPose, int scoringLevel) {
    final var scoringPoses =
        scoringLevel == 4
            ? GameConstants.reefScoringPosesL4
            : scoringLevel == 3
                ? GameConstants.reefScoringPosesL3
                : GameConstants.reefScoringPosesL2;

    final var robotXY = robotPose.getTranslation();

    // find two closest faces of the reef
    final var faceDistances = new double[6];
    for (var i = 0; i < FieldConstants.Reef.faces.length; ++i) {
      final var face = FieldConstants.fieldPose.apply(FieldConstants.Reef.faces[i]);
      faceDistances[i] = robotXY.getDistance(face.getTranslation());
    }

    int face1 = 0;
    int face2 = 1;

    if (faceDistances[0] > faceDistances[1]) {
      face1 = 1;
      face2 = 0;
    }

    for (var i = 2; i < faceDistances.length; ++i) {
      if (faceDistances[i] < faceDistances[face1]) {
        face2 = face1;
        face1 = i;
      } else if (faceDistances[i] < faceDistances[face2]) {
        face2 = i;
      }
    }

    // find the closest scoring position that is available
    Pose2d currentPose = null;
    double distance = -1;

    final var position1a = getScoringPosition(face1 * 2, scoringPoses, robotXY, scoringLevel);
    final var position1b = getScoringPosition(face1 * 2 + 1, scoringPoses, robotXY, scoringLevel);
    final var position2a = getScoringPosition(face2 * 2, scoringPoses, robotXY, scoringLevel);
    final var position2b = getScoringPosition(face2 * 2 + 1, scoringPoses, robotXY, scoringLevel);

    // target index
    int target = -1;

    if (position1a.isPresent()) {
      currentPose = position1a.get().getFirst();
      distance = position1a.get().getSecond();
      target = scoringLevel * 12 + face1 * 2;
    }

    if (position1b.isPresent() && (distance < 0 || position1b.get().getSecond() < distance)) {
      currentPose = position1b.get().getFirst();
      distance = position1b.get().getSecond();
      target = scoringLevel * 12 + face1 * 2 + 1;
    }

    if (position2a.isPresent() && (distance < 0 || position2a.get().getSecond() < distance)) {
      currentPose = position2a.get().getFirst();
      distance = position2a.get().getSecond();
      target = scoringLevel * 12 + face2 * 2;
    }

    if (position2b.isPresent() && (distance < 0 || position2b.get().getSecond() < distance)) {
      currentPose = position2b.get().getFirst();
      distance = position2b.get().getSecond();
      target = scoringLevel * 12 + face2 * 2 + 1;
    }

    return currentPose != null ? Optional.of(Pair.of(currentPose, target)) : Optional.empty();
  }

  /** Returns indexed scoring position from a list of poses and the distance to it. */
  private Optional<Pair<Pose2d, Double>> getScoringPosition(
      int index, Pose2d[] poses, Translation2d robotXY) {
    final var pose = FieldConstants.fieldPose.apply(poses[index]);
    return Optional.of(Pair.of(pose, robotXY.getDistance(pose.getTranslation())));
  }

  /**
   * Returns indexed scoring position from a list of poses and the distance to it but only if it is
   * not marked as scored at the specified level.
   */
  private Optional<Pair<Pose2d, Double>> getScoringPosition(
      int index, Pose2d[] poses, Translation2d robotXY, int level) {
    if (reefState.isPositionScored(level, index)) {
      return Optional.empty();
    }

    return getScoringPosition(index, poses, robotXY);
  }

  /**
   * Runs logic at the end of the periodic cycle.
   *
   * <p>Use this to synchronize the state with external modules, after the command scheduler runs.
   */
  public void exitPeriodic() {
    reefStateRobotPublisher.set(reefState);
  }

  /** Runs logic at the beginning of the autonomous cycle. */
  public void enterAutonomous() {
    reefState.reset();
    autoScoringStep = 0;
  }

  /** Runs logic at the beginning of the teleop cycle. */
  public void enterTeleop() {
    autoScoringStep = 0;
  }

  /** Returns observed chassis speeds at the specified timestamp. */
  public Optional<ChassisSpeeds> getSpeedsAt(long timestamp) {
    return chassisSpeeds.findAt(timestamp);
  }

  /** Returns observed pose at the specified timestamp. */
  public Optional<Pose2d> getOdometryPoseAt(long timestamp) {
    return drivetrain != null ? drivetrain.getPoseAt(timestamp) : Optional.empty();
  }

  public static final Tunable.BooleanValue visionAssist =
      Tunable.value("Control/VisionAssist", true);

  public static final Tunable.BooleanValue targetAssist =
      Tunable.value("Control/TargetAssist", true);

  public static final Tunable.BooleanValue scoreAssist = Tunable.value("Control/ScoreAssist", true);

  public static final Tunable.BooleanValue scoreL4Eject =
      Tunable.value("Control/ScoreL4Eject", true);
}
