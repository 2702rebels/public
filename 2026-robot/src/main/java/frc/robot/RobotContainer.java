// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.Logged.Importance;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.units.measure.Time;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandGenericHID;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.WheelDiameterCharacterization;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.CANduit;
import frc.robot.lib.EnergyTracker;
import frc.robot.lib.GamePhaseTracker;
import frc.robot.lib.GamePhaseTracker.Phase;
import frc.robot.lib.Tunable;
import frc.robot.subsystems.AprilTagDetector;
import frc.robot.subsystems.Climb;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.LedFeedback;
import frc.robot.subsystems.LedResource;
import frc.robot.subsystems.LedVision;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Timesync;
import frc.robot.subsystems.Turret;
import java.util.function.BooleanSupplier;

public class RobotContainer {
  private static final int kControllerDriverPort = 0;
  private static final int kControllerOperatorPort = 1;
  private static final int kButtonatorPort = 2;

  private final RobotState state = RobotState.getInstance();
  private final CommandXboxController controllerDriver =
      new CommandXboxController(kControllerDriverPort);

  private final CommandXboxController controllerOperator =
      new CommandXboxController(kControllerOperatorPort);

  private final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();
  private final GamePhaseTracker gamePhaseTracker = new GamePhaseTracker();

  private final CommandGenericHID buttonator = new CommandGenericHID(kButtonatorPort);

  private final LedResource led = new LedResource(Constants.PwmID.LED);
  private final LedFeedback ledFeedback = new LedFeedback(led, this::getDefaultLedFeedback);
  private final LedVision ledVision = new LedVision(led, this::getDefaultVisionFeedback);

  private final Timesync timesync = new Timesync();

  @Logged(name = "CANduit", importance = Importance.CRITICAL)
  private final CANduit canduit;

  @Logged(name = "AprilTagDetector", importance = Importance.CRITICAL)
  private final AprilTagDetector aprilTagDetector = new AprilTagDetector();

  @Logged(name = "Intake", importance = Importance.CRITICAL)
  public final Intake intake = new Intake(TunerConstants.kCANBus);

  @Logged(name = "Hopper", importance = Importance.CRITICAL)
  public final Hopper hopper = new Hopper(TunerConstants.kCANBus);

  @Logged(name = "Turret", importance = Importance.CRITICAL)
  public final Turret turret;

  @Logged(name = "Shooter", importance = Importance.CRITICAL)
  public final Shooter shooter = new Shooter(TunerConstants.kCANBus);

  @Logged(name = "Climb", importance = Importance.CRITICAL)
  public final Climb climb = new Climb(TunerConstants.kCANBus);

  private final AutoRoutines auto;

  private static final Trigger robotEnabled = new Trigger(DriverStation::isEnabled);

  // slows down max rotation speed for more precise movements
  private final Trigger preciseRotationActive = controllerDriver.leftStick();

  // button mappings
  private final Trigger resetOrientationButton = controllerDriver.start();
  private final Trigger brakeButton = controllerDriver.b();
  private final Trigger resetButton = controllerDriver.x().or(controllerOperator.x());
  private final Trigger enableIntakeButton = controllerDriver.a().or(controllerOperator.a());
  private final Trigger shootButton =
      controllerDriver.rightTrigger().or(controllerOperator.rightTrigger());
  private final Trigger parkHoodButton =
      controllerDriver.rightBumper().or(controllerOperator.rightBumper());
  private final Trigger climbButton =
      controllerDriver.leftBumper().or(controllerOperator.leftBumper());
  private final Trigger climbUpButton = controllerDriver.povUp().or(controllerOperator.povUp());
  private final Trigger climbDownButton =
      controllerDriver.povDown().or(controllerOperator.povDown());
  ;
  private final Trigger intakeExtend =
      controllerDriver.povRight().or(controllerOperator.povRight());
  private final Trigger intakeRetract = controllerDriver.povLeft().or(controllerOperator.povLeft());
  private final Trigger intakeAgitate =
      controllerDriver.leftTrigger().or(controllerOperator.leftTrigger());
  private final Trigger intakeBurp = controllerOperator.y().or(buttonator.button(10));
  private final Trigger hopperReverse = controllerOperator.b().or(buttonator.button(11));

  private final Trigger autoClimb = controllerDriver.y();

  // condition on when the robot is allowed to initiate shooting sequence
  @Logged(name = "Conditions/CanShoot", importance = Importance.CRITICAL)
  public final BooleanSupplier canShoot;

  // condition on when the robot is allowed to initiate auto-climbing sequence
  private final BooleanSupplier canAutoClimb;

  private final SendableChooser<Command> autoChooser = new SendableChooser<>();

  // alerts
  private static final Time kMissingAutoWinAllianceAlertThreshold = Seconds.of(1);
  private final Alert alertMissingAutoWinAlliance =
      new Alert("AUTO WINNER NOT SET!", AlertType.kError);

  private final Alert alertNoAutoRoutine = new Alert("NO AUTO ROUTINE SELECTED!", AlertType.kError);

  private final Alert alertLowBattery =
      new Alert(
          "Battery voltage is low, turn off the robot and replace the battery!",
          AlertType.kWarning);

  private final Alert alertControllerDriverDisconnected =
      new Alert(
          "Driver controller disconnected (port " + kControllerDriverPort + ").", AlertType.kError);

  private final Alert alertControllerOperatorDisconnected =
      new Alert(
          "Operator controller disconnected (port " + kControllerOperatorPort + ").",
          AlertType.kWarning);

  private final Alert alertButtonatorDisconnected =
      new Alert("Buttonator disconnected (port " + kButtonatorPort + ").", AlertType.kWarning);

  public RobotContainer(CANduit canduit) {
    this.canduit = canduit;
    turret = new Turret(TunerConstants.kCANBus, canduit);

    // must be initialized before auto routines are constructed
    // shooting sequence condition
    canShoot =
        () -> {
          if (state.isLevel() && turret.isReady()) {
            // if we are passing and not shooting, then we do not care as much about
            // flywheel (shooter) being at correct velocity
            return Field.isInPassingZone(drivetrain.getPose().getTranslation())
                ? shooter.isReadyPassing()
                : shooter.isReady();
          }

          return false;
        };

    // auto-climb condition: when FMS connected do not allow auto-climb unless in the end-game
    canAutoClimb = () -> !DriverStation.isFMSAttached() || state.getGamePhase() == Phase.EndGame;

    auto = new AutoRoutines(this, drivetrain);

    state.bind(drivetrain);
    state.setVisionStateSupplier(aprilTagDetector::getState);
    state.setGamePhaseSupplier(gamePhaseTracker::getPhase);

    turret.setTrackingSupplier(state::getTurretAngle);
    shooter.setTrackingSupplier(state::getShooterParameters);

    // [!] order matters here: we want to poll driver input first, since in most cases
    // we set intent via instant command that updates the robot state in place;
    // thus allowing triggers bound to the intent-based conditions to fire in the same
    // update cycle (this only works if the logic runs within command init method)
    configureDriverBindings();
    configureTriggers();

    // register subsystems for energy tracking
    final var energyTracker = EnergyTracker.getInstance();
    energyTracker.register("Drive", drivetrain);
    energyTracker.register("Intake", intake);
    energyTracker.register("Hopper", hopper);
    energyTracker.register("Turret", turret);
    energyTracker.register("Shooter", shooter);
    energyTracker.register("Climb", climb);

    // auto routine chooser
    autoChooser.setDefaultOption("None", Commands.none().withName("None"));
    autoChooser.addOption("Left 2 Sprints Over Bump", auto.trenchLBump2Runs());
    autoChooser.addOption("Right 2 Sprints Over Bump", auto.trenchRBump2Runs());
    autoChooser.addOption("Left 2 Sprints Over Bump and Climb", auto.trenchLBump2RunsClimb());
    autoChooser.addOption("Right 2 Sprints Over Bump and Climb", auto.trenchRBump2RunsClimb());
    autoChooser.addOption("Left 2 Sprints Over Bump [FAST]", auto.trenchLBump2RunsFast());
    autoChooser.addOption("Right 2 Sprints Over Bump [FAST]", auto.trenchRBump2RunsFast());
    autoChooser.addOption("Left 2 Sprints Over Bump [RECKLESS]", auto.trenchLBump2RunsReckless());
    autoChooser.addOption("Right 2 Sprints Over Bump [RECKLESS]", auto.trenchRBump2RunsReckless());

    SmartDashboard.putData("Auto Routine", autoChooser);
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
              () -> -smoothInput(controllerDriver.getLeftY()) * getVelocityModifier(),
              // drive left with negative X (left)
              () -> -smoothInput(controllerDriver.getLeftX()) * getVelocityModifier(),
              // drive counterclockwise with negative X (left)
              () -> -smoothInput(controllerDriver.getRightX()) * getOmegaModifier()));
    }

    resetOrientationButton.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
    brakeButton.whileTrue(drivetrain.brake());

    // run SysId routines when holding back/start and X/Y
    // note that each routine should be run exactly once in a single log
    if (Constants.Settings.sysIdBindings) {
      // controllerDriver.povRight().whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
      // controllerDriver.povLeft().whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
      // controllerDriver.povUp().whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
      // controllerDriver.povDown().whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
      controllerDriver.povLeft().whileTrue(new WheelDiameterCharacterization(drivetrain, 1));
      controllerDriver.povRight().whileTrue(new WheelDiameterCharacterization(drivetrain, -1));
    }

    drivetrain.registerTelemetry(state::updateSwerveTelemetry);

    shootButton.whileTrue(hopper.enable(canShoot));
    shootButton.whileTrue(shooter.enable());

    enableIntakeButton.onTrue(intake.enable());
    resetButton.onTrue(intake.disable());

    climbUpButton.and(() -> !climb.isEngaged()).onTrue(climb.extend());
    climbDownButton.and(() -> !climb.isEngaged()).onTrue(climb.retract());

    parkHoodButton.or(state::isShooterWithinTrench).whileTrue(shooter.parkHood());

    climbButton.and(climb::isReady).onTrue(climb.engage());
    resetButton.and(climb::isEngaged).onTrue(climb.disengage());

    intakeExtend.onTrue(intake.extend());
    intakeRetract.onTrue(intake.retract());
    intakeAgitate.and(intake::isDeployed).whileTrue(intake.agitate());
    intakeBurp.whileTrue(intake.burp());
    hopperReverse.whileTrue(hopper.reverse());

    autoClimb.and(canAutoClimb).onTrue(auto.climb().until(resetButton));

    buttonator.button(1).onTrue(state.setPassingToDefault());
    buttonator.button(2).onTrue(state.setPassingToTrenchAuto());
    buttonator.button(3).onTrue(state.setPassingToTrenchLeft());
    buttonator.button(4).onTrue(state.setPassingToTrenchRight());

    // self-check
    buttonator.button(12).and(() -> !DriverStation.isFMSAttached()).onTrue(selfCheck());
  }

  /** Use this method to configure trigger bindings. */
  private void configureTriggers() {
    // auto-initialize turret on enable
    robotEnabled.and(turret::shouldInitialize).onTrue(turret.initialize());

    RobotModeTriggers.autonomous().onTrue(Commands.runOnce(gamePhaseTracker::startPeriod));
    RobotModeTriggers.teleop().onTrue(Commands.runOnce(gamePhaseTracker::startPeriod));
    RobotModeTriggers.disabled()
        .onTrue(Commands.runOnce(gamePhaseTracker::startPeriod).ignoringDisable(true));

    RobotModeTriggers.teleop()
        .and(climb::isEngaged)
        .and(() -> state.getGamePhase() == Phase.Transition)
        .onTrue(climb.disengage());

    // auto-retract climb
    new Trigger(climb::isDisengagedButNotRetracted)
        .and(state::isAwayFromTower)
        .onTrue(climb.retract());

    // alert for missing auto win alliance
    RobotModeTriggers.teleop()
        .and(gamePhaseTracker.missingAutoWinAlliance)
        .and(
            () ->
                gamePhaseTracker.getPeriodElapsedTime().gte(kMissingAutoWinAllianceAlertThreshold))
        .whileTrue(
            Commands.startEnd(
                () -> {
                  state.missingAutoWinAlliance = true;
                  alertMissingAutoWinAlliance.set(true);
                },
                () -> {
                  state.missingAutoWinAlliance = false;
                  alertMissingAutoWinAlliance.set(false);
                }));

    // link vision assist to the master override
    new Trigger(() -> state.kVisionAssist.get())
        .onChange(
            Commands.runOnce(
                () -> {
                  final var v = state.kVisionAssist.get();
                  shooter.masterOverride = !v;
                  turret.masterOverride = !v;
                }));

    // hub to become active alert (5s prior)
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.isFMSAttached()
                    && !state.isHubActive()
                    && state.getGamePhaseRemainingTime().in(Seconds) < 5)
        .onTrue(rumble(Milliseconds.of(500)));

    // endgame alerts
    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(kEndgameAlert.get()))
        .onTrue(
            Commands.sequence(
                rumble(Milliseconds.of(200)),
                Commands.waitTime(Milliseconds.of(100)),
                rumble(Milliseconds.of(200)),
                Commands.waitTime(Milliseconds.of(100)),
                rumble(Milliseconds.of(200))));
  }

  public void periodic() {
    // alert for low battery in disabled state
    alertLowBattery.set(DriverStation.isDisabled() && state.isLowBattery());

    // alert for disconnected controllers
    alertControllerDriverDisconnected.set(
        !DriverStation.isJoystickConnected(controllerDriver.getHID().getPort()));

    alertControllerOperatorDisconnected.set(
        !DriverStation.isJoystickConnected(controllerOperator.getHID().getPort()));

    alertButtonatorDisconnected.set(
        !DriverStation.isJoystickConnected(buttonator.getHID().getPort()));

    // no auto routine selected
    alertNoAutoRoutine.set(getAutonomousCommand().getName() == "None");
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

  /** Returns feedback for LED subsystem */
  private LedFeedback.Type getDefaultLedFeedback() {
    return LedFeedback.Type.kNone;
  }

  private LedVision.Type getDefaultVisionFeedback() {
    return state.isPoseTrusted()
        ? state.hasRecentVisionPose() ? LedVision.Type.kTrusted : LedVision.Type.kTrurstedNoVision
        : LedVision.Type.kUnknown;
  }

  private Command clearOrientationLock() {
    return Commands.runOnce(drivetrain::clearOrientationLock);
  }

  /** Creates a command to rumble the controllers. */
  private Command rumble(Time time) {
    return Commands.sequence(
        Commands.runOnce(
            () -> {
              controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0);
              controllerOperator.getHID().setRumble(RumbleType.kBothRumble, 1.0);
            }),
        Commands.waitTime(time),
        Commands.runOnce(
            () -> {
              controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0);
              controllerOperator.getHID().setRumble(RumbleType.kBothRumble, 0.0);
            }));
  }

  /** Creates a command to run self-check for all subsystems. */
  private Command selfCheck() {
    return Commands.sequence(
        Commands.print("Self-check sequence started"),
        drivetrain.selfCheck(),
        intake.selfCheck(),
        hopper.selfCheck(),
        turret.selfCheck(),
        shooter.selfCheck(),
        intake.retract(), // retract intake after we are done with the shooter
        climb.selfCheck(),
        Commands.print("Self-check sequence completed"));
  }

  private boolean shouldApplyShootSlowdown() {
    if (shootButton.getAsBoolean()) {
      final var xy = drivetrain.getPose().getTranslation();
      return Field.isInShootingZone(xy) || kPassingSlowdown.get() && Field.isInPassingZone(xy);
    }

    return false;
  }

  /** Gets modifier of the rotational velocity. */
  private double getOmegaModifier() {
    return preciseRotationActive.getAsBoolean()
        ? kPreciseRotation.get()
        : shouldApplyShootSlowdown() ? kShootSlowdownRotation.get() : 1.0;
  }

  /** Gets modifier of the translational (linear) velocity. */
  private double getVelocityModifier() {
    return shouldApplyShootSlowdown() ? kShootSlowdownTranslation.get() : 1.0;
  }

  // spotless:off
  private static final Tunable.DoubleValue kEndgameAlert = Tunable.value("Endgame/Alert", 12.0);
  private static final Tunable.DoubleValue kPreciseRotation = Tunable.value("Control/PreciseRotation", 0.2);
  private static final Tunable.DoubleValue kShootSlowdownTranslation = Tunable.value("Control/ShootSlowdown/Translation", 0.3, true);
  private static final Tunable.DoubleValue kShootSlowdownRotation = Tunable.value("Control/ShootSlowdown/Rotation", 0.25, true);
  private static final Tunable.BooleanValue kPassingSlowdown = Tunable.value("Control/PassingSlowdown", false, true);
  // spotless:on
}
