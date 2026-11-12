// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import dev.doglog.DogLog;
import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.RobotState.Intent;
import frc.robot.commands.DriveToPose;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.Tunable;
import frc.robot.subsystems.AprilTagDetector;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.LedFeedback;
import frc.robot.subsystems.LedResource;
import frc.robot.subsystems.LedVision;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.ObjectDetector;
import frc.robot.subsystems.Pivot;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.Superstructure.Position;
import java.util.Map;
import java.util.Optional;
import java.util.Set;
import java.util.function.Supplier;

public class RobotContainer {
  private final RobotState state = RobotState.getInstance();

  private final Telemetry logger = new Telemetry(Constants.Settings.ctreLoggingEnabled);

  private final CommandXboxController controllerDriver = new CommandXboxController(0);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

  @Logged(name = "Elevator", importance = Importance.INFO)
  private final Elevator elevator =
      new Elevator(
          Constants.CanivoreID.MOTOR_ELEVATOR_LEADER,
          Constants.CanivoreID.MOTOR_ELEVATOR_FOLLOWER,
          TunerConstants.kCANBus);

  @Logged(name = "Pivot", importance = Importance.INFO)
  private final Pivot pivot =
      new Pivot(Constants.CanID.MOTOR_PIVOT_LEADER, Constants.CanID.MOTOR_PIVOT_FOLLOWER);

  @Logged(name = "Manipulator", importance = Importance.INFO)
  private final Manipulator manipulator =
      new Manipulator(
          Constants.CanID.MOTOR_MANIPULATOR_GUIDE, Constants.CanID.MOTOR_MANIPULATOR_CORAL);

  @Logged(name = "Climb", importance = Importance.INFO)
  private final Climb climb =
      new Climb(
          Constants.CanID.MOTOR_CLIMB_WINCH,
          Constants.DioID.SWITCH_CLIMB,
          Constants.PwmID.SERVO_CLIMB);

  @Logged(name = "Superstructure", importance = Importance.INFO)
  private final Superstructure superstructure = new Superstructure(elevator, pivot, manipulator);

  @Logged(name = "ObjectDetector", importance = Importance.INFO)
  private final ObjectDetector objectDetector = new ObjectDetector();

  @Logged(name = "AprilTagDetector", importance = Importance.INFO)
  private final AprilTagDetector aprilTagDetector = new AprilTagDetector();

  private final LedResource led = new LedResource(Constants.PwmID.LED);
  private final LedFeedback ledFeedback = new LedFeedback(led, this::getDefaultLedFeedback);
  private final LedVision ledVision = new LedVision(led, this::getDefaultVisionFeedback);

  // startup validation checklist
  @Logged(name = "Checklist/PivotEncoder", importance = Importance.CRITICAL)
  private boolean pivotEncoderValid = false;

  @Logged(name = "State/HasOrientationLock", importance = Importance.INFO)
  private final Trigger hasOrientationLock = new Trigger(drivetrain::hasOrientationLock);

  private static final Trigger disabled = new Trigger(() -> false);
  private static final Trigger isAutonomousMode = new Trigger(DriverStation::isAutonomousEnabled);

  // trigger indicating that superstructure is cleared for deployment;
  // used with delayed deployment during auto driving
  private final Trigger superstructureReady = (state.isAuto.and(superstructure.isDelayed)).negate();

  private final AutoFactory autoFactory;
  private final AutoRoutines autoRoutines;
  private final AutoChooser autoChooser = new AutoChooser();

  // slows down max rotation speed for more precise movements
  private final Trigger preciseRotationActive = controllerDriver.leftStick();

  // button mappings
  private final Trigger resetOrientationButton = controllerDriver.start();
  private final Trigger brakeButton = controllerDriver.b();
  private final Trigger resetIntentButton = controllerDriver.x();
  private final Trigger floorIntakeButton = controllerDriver.leftTrigger();

  private final Trigger hoverButton =
      Constants.Settings.sysIdBindings ? disabled : controllerDriver.povUp();
  private final Trigger coralEjectButton =
      Constants.Settings.sysIdBindings
          ? disabled
          : controllerDriver.povDown().and(state.shouldClimbReady.negate());

  private final Trigger climbReadyButton = controllerDriver.a();
  private final Trigger climbEngageButton =
      controllerDriver.a().and(state.shouldClimbReady).and(climb.isLocked);
  private final Trigger climbResetButton =
      controllerDriver.povDown().and(state.shouldClimbReady).and(climb.isLocked.negate());

  private final Trigger scoreButton = controllerDriver.rightTrigger();
  private final Trigger scoreManualButton = controllerDriver.rightBumper();
  private final Trigger scoreAlgaeNetButton = controllerDriver.leftBumper();

  // algae manipulation
  private final Trigger algaeIntakeL2Button =
      Constants.Settings.sysIdBindings ? disabled : controllerDriver.povLeft();
  private final Trigger algaeIntakeL3Button =
      Constants.Settings.sysIdBindings ? disabled : controllerDriver.povRight();

  private final Trigger algaeIntakeGroundButton = controllerDriver.y();

  private static final double kDriverTolerance = 0.05;

  // trigger for detecting driver input
  private final Trigger hasDriverInput =
      new Trigger(
          () ->
              DriverStation.isTeleopEnabled()
                  && (!MathUtil.isNear(0, controllerDriver.getLeftX(), kDriverTolerance)
                      || !MathUtil.isNear(0, controllerDriver.getLeftY(), kDriverTolerance)
                      || !MathUtil.isNear(0, controllerDriver.getRightX(), kDriverTolerance)));

  // trigger for scoring coral auto or manually
  // NOTE: change to "isAutonomousMode.or(isAuto)" to allow robot to auto score,
  // right now it will wait for the score button to be pressed in teleop
  private final Trigger shouldInitiateCoralSequence =
      isAutonomousMode
          .or(state.isAuto.and(state.isAutoScore))
          .and(state.atScoringPose)
          .debounce(0.04)
          .or(scoreButton.debounce(0.5, DebounceType.kFalling));

  private final Trigger shouldInitiateL4CoralSequence =
      isAutonomousMode
          .and(state.atScoringPose)
          .debounce(0.1)
          .or(scoreButton.debounce(0.5, DebounceType.kFalling));

  public RobotContainer() {
    state.bind(drivetrain);

    // bind suppliers to the state
    state.setVisionStateSupplier(aprilTagDetector::getState);
    state.setDetectedObjectSupplier(objectDetector::getDetectedObject);

    autoFactory = drivetrain.createAutoFactory();
    autoRoutines = new AutoRoutines(autoFactory, drivetrain, superstructure);

    autoChooser.addRoutine(
        "DriveForward", () -> autoRoutines.naive("Drive forward", driveForward(Inches.of(24))));

    autoChooser.addRoutine("Score L4 (J)", () -> autoRoutines.scoreL4x('J'));
    autoChooser.addRoutine("Score L4 (I)", () -> autoRoutines.scoreL4x('I'));
    autoChooser.addRoutine("Score L4 (H)", () -> autoRoutines.scoreL4x('H'));
    autoChooser.addRoutine("Score L4 (G)", () -> autoRoutines.scoreL4x('G'));
    autoChooser.addRoutine("Score L4 (F)", () -> autoRoutines.scoreL4x('F'));
    autoChooser.addRoutine("Score L4 (E)", () -> autoRoutines.scoreL4x('E'));

    autoChooser.addRoutine("Score L4 (I,L) Left Side", autoRoutines::scoreL4IL);
    autoChooser.addRoutine("Score L4 (F,C) Right Side", autoRoutines::scoreL4FC);
    autoChooser.addRoutine("Score L4 (J,L) Left Side", autoRoutines::scoreL4JL);
    autoChooser.addRoutine("Score L4 (E,C) Right Side", autoRoutines::scoreL4EC);

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // [!] order matters here: we want to poll driver input first, since in most cases
    // we set intent via instant command that updates the robot state in place;
    // thus allowing triggers bound to the intent-based conditions to fire in the same
    // update cycle (this only works if the logic runs within command init method)
    configureDriverBindings();
    configureTriggers();
  }

  /** Smooth out control input by squaring. */
  private double smoothInput(double value) {
    final var v = MathUtil.clamp(value, -1, 1);
    return Math.signum(value) * v * v;
  }

  /** Use this method to configure driver controller bindings. */
  private void configureDriverBindings() {
    if (!Constants.Settings.drivingDisabled) {
      // note that X is defined as forward according to WPILib convention,
      // and Y is defined as to the left according to WPILib convention.
      drivetrain.setDefaultCommand(
          // drivetrain will execute this command periodically
          drivetrain.driveTeleop(
              // drive forward with negative Y (forward)
              () -> -smoothInput(controllerDriver.getLeftY()),
              // drive left with negative X (left)
              () -> -smoothInput(controllerDriver.getLeftX()),
              // drive counterclockwise with negative X (left)
              () -> -smoothInput(controllerDriver.getRightX()) * getOmegaModifier()));
    }

    resetOrientationButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    brakeButton.whileTrue(drivetrain.brake());

    // run SysId routines when holding back/start and X/Y
    // note that each routine should be run exactly once in a single log
    if (Constants.Settings.sysIdBindings) {
      controllerDriver.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      controllerDriver.povLeft().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      controllerDriver.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      controllerDriver.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
      // controllerDriver.povLeft().whileTrue(new WheelDiameterCharacterization(drivetrain, 1));
      // controllerDriver.povRight().whileTrue(new WheelDiameterCharacterization(drivetrain, -1));
    }

    drivetrain.registerTelemetry(logger::update);

    resetIntentButton.onTrue(state.setIntent(Intent.kNone));

    // coral intake
    floorIntakeButton.onTrue(state.setIntent(Intent.kCoralFloorIntake));

    // no more station intake with new intake design
    // stationIntakeButton.onTrue(state.setIntent(Intent.kCoralStationIntake));

    // coral scoring
    scoreButton.and(superstructure.hasCoral).onTrue(state.setIntent(Intent.kCoralScoreAuto));
    scoreManualButton.and(superstructure.hasCoral).onTrue(state.setIntent(Intent.kCoralScoreL1));

    // algae scoring
    scoreAlgaeNetButton.and(superstructure.hasAlgae).onTrue(state.setIntent(Intent.kAlgaeScoreNet));
    scoreManualButton.and(superstructure.hasAlgae).onTrue(state.setIntent(Intent.kAlgaeEject));

    algaeIntakeGroundButton.onTrue(state.setIntent(Intent.kAlgaeIntakeGround));
    algaeIntakeL2Button.onTrue(state.setIntent(Intent.kAlgaeIntakeL2));
    algaeIntakeL3Button.onTrue(state.setIntent(Intent.kAlgaeIntakeL3));

    coralEjectButton.onTrue(state.setIntent(Intent.kCoralEject));
    coralEjectButton.onFalse(state.setIntent(Intent.kNone));

    hoverButton.onTrue(state.setIntent(Intent.kHover));
    hoverButton.onFalse(state.setIntent(Intent.kNone));

    // climbing
    climbReadyButton.onTrue(state.setIntent(Intent.kClimbReady));
    climbEngageButton.onTrue(state.setIntent(Intent.kClimbEngage));
    climbResetButton.onTrue(state.setIntent(Intent.kNone).andThen(climb.reset()));
  }

  /** Use this method to configure trigger bindings. */
  private void configureTriggers() {
    // intent-driven triggers

    // eject coral (emergency mechanism does not check for coral presence)
    state.shouldEjectCoral.and(superstructure.canEject).whileTrue(manipulator.ejectCoral());
    state.shouldEjectCoral.onTrue(superstructure.setPosition(Position.kL1));

    // intake algae
    state
        .shouldIntakeAlgae
        .and(superstructure.hasAlgae.negate())
        .and(superstructure.canIntakeAlgae)
        .whileTrue(manipulator.intakeAlgae());

    state
        .shouldIntakeAlgaeL2
        .and(superstructure.hasAlgae.negate())
        .onTrue(superstructure.setPosition(Position.kL2Algae));

    state
        .shouldIntakeAlgaeL3
        .and(superstructure.hasAlgae.negate())
        .onTrue(superstructure.setPosition(Position.kL3Algae));

    state
        .shouldIntakeAlgaeGround
        .and(superstructure.hasAlgae.negate())
        .onTrue(superstructure.setPosition(Position.kGroundAlgae));

    // hold algae while presence is set unless ejecting
    superstructure
        .hasAlgae
        .and(manipulator.isEjectingAlgae.negate())
        .whileTrue(manipulator.holdAlgae());

    // eject/score algae (processor)
    state
        .shouldEjectAlgae
        .and(superstructure.hasAlgae)
        .onTrue(superstructure.setPosition(Position.kEjectAlgae));

    state
        .shouldEjectAlgae
        .and(superstructure.hasAlgae)
        .and(superstructure.canEjectAlgae)
        .and(scoreManualButton)
        .onTrue(ejectAlgaeSequence());

    // score algae (net)
    state
        .shouldScoreAlgaeNet
        .and(superstructure.hasAlgae)
        .onTrue(superstructure.setPosition(Position.kNet).andThen(setBargeOrientationLock()))
        .and(() -> GameConstants.isAtNetScoringPosition(state.getRobotPose()))
        .whileTrue(ledFeedback.signalAttainment());

    state
        .shouldScoreAlgaeNet
        .and(superstructure.hasAlgae)
        .and(superstructure.canScoreAlgae)
        .and(scoreAlgaeNetButton)
        .onTrue(ejectAlgaeSequence());

    // intake coral from the floor
    state
        .shouldIntakeCoralFloor
        .and(superstructure.hasCoral.negate())
        .onTrue(superstructure.setPosition(Position.kFloor));

    state
        .shouldIntakeCoralFloor
        .and(superstructure.hasCoral.negate())
        .and(superstructure.canFloorIntake)
        .whileTrue(manipulator.intakeCoral().unless(this::isIntakeDisabled));

    // reindex coral after acquisition
    state.shouldIntakeCoral.and(superstructure.hasCoral).onTrue(manipulator.reindexCoral());

    // reset intent once coral has been acquired
    state
        .shouldIntakeCoral
        .and(superstructure.hasCoral)
        .onTrue(state.setIntent(Intent.kCoralReady));

    // rumble once the coral has been acquired
    superstructure
        .hasCoral
        .onTrue(rumble(Milliseconds.of(300)))
        .onTrue(ledFeedback.signalSuccess());

    // score coral at L1
    state
        .shouldScoreCoralL1
        .and(superstructure.hasCoral)
        .onTrue(superstructure.setPosition(Position.kL1));

    state
        .shouldScoreCoralL1
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreL1)
        .and(scoreManualButton.debounce(0.5, DebounceType.kFalling))
        .whileTrue(manipulator.ejectCoralSlow());

    // score coral via auto target
    state.isAuto.and(state.hasScoringPose).onTrue(driveAutoScore().until(hasDriverInput));

    // when in autonomous mode brake momentarily at the end of drive to prevent robot
    // from moving once the scoring position has been reached
    isAutonomousMode
        .and(state.hasScoringPose)
        .and(state.atScoringPose)
        .onTrue(drivetrain.brake().withTimeout(Milliseconds.of(150)));

    // reset auto mode once driver input is received
    state.isAuto.and(hasDriverInput).onTrue(state.resetAuto());

    // scoring L2, L3 (elevate to L2 and L3 as we are driving)
    state
        .shouldScoreCoralL2
        .and(superstructure.hasCoral)
        .and(superstructureReady)
        .onTrue(superstructure.setPosition(Position.kL2));

    state
        .shouldScoreCoralL2
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreL2)
        .and(shouldInitiateCoralSequence)
        .whileTrue(state.markScoringTarget().andThen(manipulator.ejectCoral()));

    state
        .shouldScoreCoralL3
        .and(superstructure.hasCoral)
        .and(superstructureReady)
        .onTrue(superstructure.setPosition(Position.kL3));

    state
        .shouldScoreCoralL3
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreL3)
        .and(shouldInitiateCoralSequence)
        .whileTrue(state.markScoringTarget().andThen(manipulator.ejectCoral()));

    // scoring L4
    state
        .shouldScoreCoralL4
        .and(superstructure.hasCoral)
        .and(superstructureReady)
        .onTrue(superstructure.setPosition(Position.kL4));

    state
        .shouldScoreCoralL4
        .and(superstructure.hasCoral)
        .and(superstructure.canScoreL4)
        .and(shouldInitiateL4CoralSequence)
        .onTrue(state.resetScoringPose().andThen(state.setIntent(Intent.kCoralScoreL4Final)));

    configureL4Sequence();

    // reset intent once coral has been scored
    state
        .shouldScoreCoralL1
        .or(state.shouldScoreCoralL2)
        .or(state.shouldScoreCoralL3)
        .and(superstructure.hasCoral.negate())
        .onTrue(
            // backup a little bit to avoid hitting corals on the way down
            driveForward(Inches.of(-5)).andThen(state.setIntent(Intent.kNone)));

    // hover
    state.shouldHover.onTrue(superstructure.setPosition(Position.kHover));

    // climb
    state.shouldClimbReady.onTrue(
        superstructure.setPosition(Position.kClimb).andThen(climb.ready()));

    // rumble once climb lock has been acquired;
    // and signal that we are the position to climb
    state
        .shouldClimbReady
        .and(climb.isLocked)
        .onTrue(rumble(Milliseconds.of(300)))
        .onTrue(ledFeedback.signalSuccess())
        .and(() -> GameConstants.isAtClimbPosition(state.getRobotPose()))
        .whileTrue(ledFeedback.signalAttainment());

    // automatically eject coral if we have one as we engage climber
    state
        .shouldClimbEngage
        .and(climb.isLocked)
        .onTrue(
            drivetrain
                .wheelsForward()
                .withTimeout(Milliseconds.of(100))
                .andThen(
                    Commands.either(
                        climb
                            .engage()
                            .alongWith(
                                manipulator.ejectCoral().until(superstructure.hasCoral.negate())),
                        climb.engage(),
                        superstructure.hasCoral)));

    // elevator safety to deal with game pieces potentially stuck within frame:
    // if stall is detected when idle (stowed mode) and elevator is below
    // threshold then automatically go into hover mode
    state
        .idle
        .or(state.coralReady)
        .and(superstructure.isElevatorStalled)
        .and(() -> elevator.getPosition() < 32)
        .onTrue(state.setIntent(Intent.kHover));

    // ready state when having coral acquired
    state.coralReady.onTrue(superstructure.setPosition(Position.kReady));

    // [default behavior] stow when no intent is specified
    state.idle.whileTrue(superstructure.setPosition(Position.kStowed));

    //
    // special handling for situations when elevator is above 54", e.g.
    // when we are stuck on the reef while trying to score L4 and are trying
    // to abort -- first go up, then back up, before stowing the pivot
    // state.idle.whileTrue(
    //     Commands.either(
    //         Commands.sequence(
    //             superstructure.setHeightOverride(60),
    //             driveForward(Inches.of(-10)),
    //             superstructure.resetHeightOverride(),
    //             superstructure.setPosition(Position.kStowed)),
    //         superstructure.setPosition(Position.kStowed),
    //         () -> elevator.isAbovePosition(52)));

    // always clear orientation lock when do not have algae
    hasOrientationLock.and(superstructure.hasAlgae.negate()).onTrue(clearOrientationLock());

    // LED
    state.atScoringPose.whileTrue(ledFeedback.signalReady());
    state.isAuto.whileTrue(ledVision.signalAssist());

    // endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(kEndgameAlert1.get()))
        .onTrue(rumble(Milliseconds.of(500)));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(kEndgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                rumble(Milliseconds.of(200)),
                Commands.waitTime(Milliseconds.of(100)),
                rumble(Milliseconds.of(200)),
                Commands.waitTime(Milliseconds.of(100)),
                rumble(Milliseconds.of(200))));
  }

  /** Returns feedback for LED subsystem */
  private LedFeedback.Type getDefaultLedFeedback() {
    switch (state.intent) {
      case kCoralScoreL1:
      case kCoralScoreL2:
      case kCoralScoreL3:
      case kCoralScoreL4:
        return LedFeedback.Type.kScoringCoral;
      case kCoralFloorIntake:
      case kCoralStationIntake:
        return LedFeedback.Type.kIntakingCoral;
      case kAlgaeScoreNet:
      case kAlgaeEject:
        return LedFeedback.Type.kScoringAlgae;
      case kAlgaeIntakeL2:
      case kAlgaeIntakeL3:
      case kAlgaeIntakeGround:
        return LedFeedback.Type.kIntakingAlgae;
      case kClimbReady:
        return LedFeedback.Type.kClimbing;
      default:
        if (superstructure.hasCoral.getAsBoolean()) {
          return LedFeedback.Type.kHoldingCoral;
        }
        if (superstructure.hasAlgae.getAsBoolean()) {
          return LedFeedback.Type.kHoldingAlgae;
        }
        return LedFeedback.Type.kNone;
    }
  }

  private LedVision.Type getDefaultVisionFeedback() {
    return state.isPoseTrusted.getAsBoolean() ? LedVision.Type.kTrusted : LedVision.Type.kUnknown;
  }

  /** Estimates distance drive time based on the max acceleration and speed. */
  private Time estimateDriveTime(
      Distance distance, LinearVelocity maxVelocity, LinearAcceleration maxAcceleration) {
    final double maxAccelerationTimeSeconds =
        maxVelocity.in(MetersPerSecond) / maxAcceleration.in(MetersPerSecondPerSecond);

    final Distance maxAccelerationOnlyDistance =
        Meters.of(
            Math.pow(maxAccelerationTimeSeconds, 2) * maxAcceleration.in(MetersPerSecondPerSecond));

    // acceleration/decelaration only motion
    if (distance.lte(maxAccelerationOnlyDistance)) {
      return Seconds.of(
          Math.sqrt(distance.in(Meters) / maxAcceleration.in(MetersPerSecondPerSecond)) * 2);
    }

    return Seconds.of(
        2 * maxAccelerationTimeSeconds
            + (distance.in(Meters) - maxAccelerationOnlyDistance.in(Meters))
                / maxVelocity.in(MetersPerSecond));
  }

  /** Creates a command to drive to the specified pose along with the estimated flight time. */
  private Pair<Command, Time> createTimedDriveToPose(
      Supplier<Optional<Pose2d>> pose,
      LinearVelocity maxVelocity,
      LinearAcceleration maxAcceleration) {
    final var currentPose = drivetrain.getPose();
    final var targetPoseSupplied = pose.get();

    if (!targetPoseSupplied.isPresent()) {
      return Pair.of(Commands.none(), Seconds.zero());
    }

    final var targetPose = targetPoseSupplied.get();
    final var distance = targetPose.getTranslation().getDistance(currentPose.getTranslation());

    return Pair.of(
        new DriveToPose(drivetrain, pose, state::getRobotPose, maxVelocity, maxAcceleration),
        estimateDriveTime(Meters.of(distance), maxVelocity, maxAcceleration));
  }

  /** Creates a pose in front of the given pose offset by the specified distance. */
  private static Pose2d getPoseInFront(Pose2d pose, Distance distance) {
    final var vector = new Translation2d(distance.in(Meters), 0).rotateBy(pose.getRotation());
    return new Pose2d(pose.getTranslation().plus(vector), pose.getRotation());
  }

  private static final LinearVelocity maxAutoVelocity =
      Constants.DriveConstants.maxSpeed.times(Constants.DriveConstants.kAutoSpeedModifier);

  private static final LinearAcceleration maxAutoAcceleration =
      Constants.DriveConstants.maxAcceleration.times(Constants.DriveConstants.kAutoSpeedModifier);

  /** Creates a command to drive forward/backward fixed distance with driver interruption. */
  private Command driveForward(Distance distance) {
    // timeout based on the distance, max acceleration and speed (+20%)
    final var timeout =
        estimateDriveTime(distance, maxAutoVelocity, maxAutoAcceleration).times(1.2);

    return new InstantCommand(
            () -> state.setTargetPose(getPoseInFront(state.getRobotPose(), distance)))
        .andThen(
            new DriveToPose(
                    drivetrain,
                    state::getTargetPose,
                    state::getRobotPose,
                    maxAutoVelocity,
                    maxAutoAcceleration)
                .withTimeout(timeout)
                .until(hasDriverInput));
  }

  private static final Map<RobotState.Intent, Time> kSuperstructureTimeToTarget =
      Map.of(
          RobotState.Intent.kCoralScoreL2,
          Seconds.of(1),
          RobotState.Intent.kCoralScoreL3,
          Seconds.of(1.3),
          RobotState.Intent.kCoralScoreL4,
          Seconds.of(1.5));

  // normal vector corresponding to a rotation of zero degrees
  private static final Translation2d kUnit = new Translation2d(1, 0);

  /**
   * Determines whether we are inside the half-plane on the target side, meaning that robot has to
   * move backwards/sideways to get there.
   *
   * <p>Assume that target pose points into the half plane.
   *
   * @param target Target pose defining the half plane
   * @param current Current pose of the robot
   * @param threshold Offset to the half plane boundary
   */
  private static boolean isInsideHalfPlane(Pose2d target, Pose2d current, double threshold) {
    final var normal = kUnit.rotateBy(target.getRotation());
    final var a = normal.getX();
    final var b = normal.getY();
    final var c = -(a * target.getX() + b * target.getY());

    return a * current.getX() + b * current.getY() + c >= threshold;
  }

  private static final double kRepositionThreshold = Units.inchesToMeters(-4);
  private static final Distance kRepositionDistance = Inches.of(-6);

  /** Returns repositioned scoring pose in front of the target. */
  private Optional<Pose2d> getRepositionedScoringPose() {
    final var targetPose = state.getScoringPose();
    return targetPose.isPresent()
        ? Optional.of(getPoseInFront(targetPose.get(), kRepositionDistance))
        : Optional.empty();
  }

  /**
   * Crates a command sequence to drive to position and prepare to score at the L2,L3,L4.
   *
   * <p>Determines timings for driving and commanding superstructure based on the current robot's
   * position. IMPORTANT: creates a side-effect of setting superstructure delay flag.
   */
  private Command driveAutoScore() {
    return Commands.defer(
        () -> {
          final var modifier =
              (isAutonomousMode.getAsBoolean()
                      ? Constants.DriveConstants.kAutonomousModeModifier
                      : 1)
                  * (state.intent == Intent.kCoralScoreL4
                      ? Constants.DriveConstants.kAutoL4SpeedModifier
                      : Constants.DriveConstants.kAutoSpeedModifier);

          final var maxVelocity = Constants.DriveConstants.maxSpeed.times(modifier);
          final var maxAcceleration = Constants.DriveConstants.maxAcceleration.times(modifier);

          final var superstructureTime = kSuperstructureTimeToTarget.get(state.intent);

          // if we are inside the half-plane reposition to a better pose in front of the target
          final var targetPoseSupplied = state.getScoringPose();
          if (targetPoseSupplied.isPresent()
              && isInsideHalfPlane(
                  targetPoseSupplied.get(), drivetrain.getPose(), kRepositionThreshold)) {

            superstructure.setDelayed(true);
            return Commands.sequence(
                    new DriveToPose(
                            drivetrain,
                            this::getRepositionedScoringPose,
                            state::getRobotPose,
                            maxVelocity,
                            maxAcceleration)
                        .withTimeout(3),
                    Commands.runOnce(() -> superstructure.setDelayed(false)),
                    Commands.waitTime(
                        superstructureTime != null ? superstructureTime : Seconds.zero()),
                    new DriveToPose(
                        drivetrain,
                        state::getScoringPose,
                        state::getRobotPose,
                        maxVelocity,
                        maxAcceleration))
                .finallyDo(() -> superstructure.setDelayed(false));
          }

          final var pair =
              createTimedDriveToPose(state::getScoringPose, maxVelocity, maxAcceleration);

          final var driveCommand = pair.getFirst();
          final var driveTime = pair.getSecond();

          if (superstructureTime == null) {
            return driveCommand;
          }

          // if the superstructure time is more than driving time, delay driving by the delta
          // to alow superstructure time to extend first, otherwise delay superstructure
          if (superstructureTime.gt(driveTime)) {
            return Commands.waitTime(superstructureTime.minus(driveTime)).andThen(driveCommand);
          } else if (driveTime.gt(superstructureTime)) {
            // this is a side-effect that impacts other triggers resolution
            superstructure.setDelayed(true);
            return driveCommand
                .alongWith(
                    Commands.waitTime(driveTime.minus(superstructureTime))
                        .andThen(Commands.runOnce(() -> superstructure.setDelayed(false))))
                .finallyDo(() -> superstructure.setDelayed(false));
          } else {
            return driveCommand;
          }
        },
        Set.of(drivetrain));
  }

  /** Creates a command sequence to eject algae and reset intent and presence. */
  private Command ejectAlgaeSequence() {
    return manipulator.ejectAlgae().withTimeout(3).andThen(state.setIntent(Intent.kNone));
  }

  private void configureL4Sequence() {
    // try to score by lowering the elevator to the "L4 scored" position until
    // we successfully reach the position or stall is detected, in case of the latter
    // go into retry mode
    state
        .shouldScoreCoralL4Final
        .and(state.shouldScoreL4Eject.negate().or(isAutonomousMode))
        .onTrue(
            superstructure
                .maintainPosition(Position.kL4Scored)
                .raceWith(
                    Commands.waitUntil(superstructure::isDoneScoringL4),
                    Commands.waitUntil(superstructure.isElevatorStalled))
                .andThen(
                    Commands.either(
                        state.setIntent(Intent.kCoralScoreL4Retry),
                        Commands.sequence(
                            state.markScoringTarget(),
                            driveForward(Inches.of(-10)),
                            state.setIntent(Intent.kNone)),
                        superstructure.isElevatorStalled)));

    // score L4 by ejecting downwards (with new intake)
    state
        .shouldScoreCoralL4Final
        .and(state.shouldScoreL4Eject.and(isAutonomousMode.negate()))
        .onTrue(
            Commands.sequence(
                manipulator.ejectCoralReverse().until(superstructure.hasCoral.negate()),
                state.markScoringTarget(),
                driveForward(Inches.of(-10)),
                state.setIntent(Intent.kNone)));

    // manual retry does not capture drive subsystem and allows operator
    // to reposition while maintaining elevator above the reef and waiting
    // for an input to rescore
    state
        .shouldScoreCoralL4Retry
        .and(isAutonomousMode.negate())
        .onTrue(
            superstructure
                .maintainPosition(Position.kL4Retry)
                .until(scoreButton)
                .andThen(state.setIntent(Intent.kCoralScoreL4Final)));

    // autonomous retry captures drive subsystem to automatically attempt
    // to reposition the robot and retry the scoring sequence
    state
        .shouldScoreCoralL4Retry
        .and(isAutonomousMode)
        .and(state::canAutoRetryL4)
        .onTrue(
            Commands.sequence(
                superstructure
                    .maintainPosition(Position.kL4Retry)
                    .withTimeout(Milliseconds.of(200)),
                new DriveToPose(drivetrain, state::getAutoRetryL4Pose, state::getRobotPose)
                    .withTimeout(Milliseconds.of(400)),
                Commands.waitTime(Milliseconds.of(200)),
                state.setIntent(Intent.kCoralScoreL4Final)));
  }

  private Command setBargeOrientationLock() {
    return Commands.runOnce(() -> drivetrain.setOrientationLock(Rotation2d.k180deg));
  }

  private Command clearOrientationLock() {
    return Commands.runOnce(drivetrain::clearOrientationLock);
  }

  public Command getAutonomousCommand() {
    // run the routine selected from the auto chooser
    return autoChooser.selectedCommand();
  }

  /** Returns intake disabled state (autonomous moe only). */
  private boolean isIntakeDisabled() {
    return DriverStation.isAutonomousEnabled() && superstructure.shouldDisableIntake.getAsBoolean();
  }

  public void disableIntake() {
    superstructure.disableIntake();
  }

  public void enableIntake() {
    superstructure.enableIntake();
  }

  /** Runs validation checklist at the startup. Must be invoked from the robotInit(). */
  public void initValidation() {
    // validate that pivot absolute encoder reading matches the expected angle
    // of the mechanism in its starting configuration; this ensure that encoder
    // is properly zeroed at the start of the match
    pivotEncoderValid = pivot.isAtAngle(Constants.initialPivotAngle);

    final var failed = !pivotEncoderValid;

    if (failed) {
      DogLog.logFault("Startup checklist failed", AlertType.kError);
    }
  }

  /** Creates a command to rumble the controllers. */
  private Command rumble(Time time) {
    return Commands.sequence(
        Commands.runOnce(() -> controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0)),
        Commands.waitTime(time),
        Commands.runOnce(() -> controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0)));
  }

  /** Gets modifier of the rotational velocity when precise rotation mode is active. */
  private double getOmegaModifier() {
    return preciseRotationActive.getAsBoolean() ? kPreciseRotation.get() : 1;
  }

  // spotless:off
  private static final Tunable.DoubleValue kEndgameAlert1 = Tunable.value("Endgame/Alert1", 30.0);
  private static final Tunable.DoubleValue kEndgameAlert2 = Tunable.value("Endgame/Alert2", 10.0);
  private static final Tunable.DoubleValue kPreciseRotation = Tunable.value("Control/PreciseRotation", 0.2);
  // spotless:on
}
