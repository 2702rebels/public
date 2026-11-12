package frc.robot;

import com.choreo.lib.ChoreoTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.CommandsFactory;
import frc.robot.commands.CommandsFactory.LedMode;
import frc.robot.commands.DriveTrajectory;
import frc.robot.commands.FeedForwardCharacterization;
import frc.robot.subsystems.climb.Climb;
import frc.robot.subsystems.climb.ClimbIO;
import frc.robot.subsystems.climb.ClimbIOSim;
import frc.robot.subsystems.climb.ClimbIOSparkMax;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.subsystems.drive.GyroIO;
import frc.robot.subsystems.drive.GyroIOPigeon2;
import frc.robot.subsystems.drive.ModuleIO;
import frc.robot.subsystems.drive.ModuleIOSim;
import frc.robot.subsystems.drive.ModuleIOSparkMax;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.intake.IntakeIO;
import frc.robot.subsystems.intake.IntakeIOSim;
import frc.robot.subsystems.intake.IntakeIOSparkMax;
import frc.robot.subsystems.led.Led;
import frc.robot.subsystems.led.LedConstants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.pivot.PivotIO;
import frc.robot.subsystems.pivot.PivotIOSim;
import frc.robot.subsystems.pivot.PivotIOSparkMax;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterIO;
import frc.robot.subsystems.shooter.ShooterIOSim;
import frc.robot.subsystems.shooter.ShooterIOSparkMax;
import frc.robot.subsystems.vision.AprilTagDetector;
import frc.robot.subsystems.vision.NoteDetector;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.TrajectoryLoader;
import java.util.Map;
import java.util.function.Function;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class RobotContainer {
  // subsystems
  private final Drive drive;
  private final NoteDetector noteDetector = new NoteDetector();
  private final AprilTagDetector aprilTagDetector = new AprilTagDetector();
  private final Intake intake;
  private final Pivot pivot;
  private final Shooter shooter;
  private final Led led = new Led(LedConstants.LENGTH, LedConstants.PWM_PORT);
  private final Climb climb;

  // factory for common commands
  private final CommandsFactory commands;

  // loaded choreo trajectories
  private final Map<String, ChoreoTrajectory> trajectories;

  // driver controller
  private final CommandXboxController controllerDriver = new CommandXboxController(0);

  // dashboard inputs
  private final LoggedDashboardChooser<Command> autoChooser =
      new LoggedDashboardChooser<>("Auto Choices");

  // spotless:off
  private static final LoggedTunableNumber endgameAlert1 = new LoggedTunableNumber("Endgame/Alert 1", 30.0);
  private static final LoggedTunableNumber endgameAlert2 = new LoggedTunableNumber("Endgame/Alert 2", 10.0);

  private static final LoggedTunableNumber shooterSubwooferSpeed = new LoggedTunableNumber("Shooter/Subwoofer Speed", 3000);
  // spotless:on

  // pre-created command instances to be scheduled manually
  // when the robot is in the disabled state
  private Command commandLedLogging;
  private Command commandLedDisabled;

  public RobotContainer() {
    switch (Constants.currentMode) {
      case REAL:
        // real robot, instantiate hardware IO implementations
        drive =
            new Drive(
                new GyroIOPigeon2(CanID.GYRO_CONTROLLER, true),
                (Integer index) ->
                    new ModuleIOSparkMax(
                        DriveConstants.moduleConfigs[index],
                        DriveConstants.moduleAngularOffsets[index],
                        DriveConstants.drivetrainConfig));

        intake = new Intake(new IntakeIOSparkMax());
        pivot = new Pivot(new PivotIOSparkMax());
        shooter = new Shooter(new ShooterIOSparkMax());
        climb = new Climb(new ClimbIOSparkMax());
        break;

      case SIM:
        // sim robot, instantiate physics sim IO implementations
        drive = new Drive(new GyroIO() {}, (Integer index) -> new ModuleIOSim());
        intake = new Intake(new IntakeIOSim());
        pivot = new Pivot(new PivotIOSim());
        shooter = new Shooter(new ShooterIOSim());
        climb = new Climb(new ClimbIOSim());
        break;

      default:
        // replayed robot, disable IO implementations
        drive = new Drive(new GyroIO() {}, (Integer index) -> new ModuleIO() {});
        intake = new Intake(new IntakeIO() {});
        pivot = new Pivot(new PivotIO() {});
        shooter = new Shooter(new ShooterIO() {});
        climb = new Climb(new ClimbIO() {});
        break;
    }

    // load trajectories from the file system
    trajectories = TrajectoryLoader.loadChoreoTrajectories();

    // commands factory (must come before bindings, etc.)
    commands =
        new CommandsFactory(
            drive, led, intake, pivot, shooter, climb, aprilTagDetector, noteDetector);

    commandLedDisabled = commands.led(LedMode.DISABLED);
    commandLedLogging = commands.led(LedMode.LOGGING);

    configureBindings();
    configureTriggers();
    configureAutoRoutines();
    configureSmartDashboard();
  }

  /**
   * Creates new command that drives specified trajectory with intake running until game piece is
   * acquired.
   */
  private Command driveAndIntake(String trajectoryName, double timeoutSeconds) {
    return new DriveTrajectory(trajectories.get(trajectoryName), false, true, drive)
        .until(shooter::isFrontBeamTripped)
        .alongWith(commands.intake())
        .withTimeout(timeoutSeconds);
  }

  /**
   * Creates new command that drives specified trajectory then attempts to locate the note with
   * vision and drives towards it with intake running until game piece is acquired.
   */
  private Command driveThenIntake(String trajectoryName, double timeoutSeconds) {
    return new DriveTrajectory(trajectories.get(trajectoryName), false, true, drive)
        .andThen(Commands.waitSeconds(0.06)) // note detection vision has a perceived lag of 60ms
        .andThen(commands.driveToNote().deadlineWith(commands.intake()))
        .withTimeout(timeoutSeconds);
  }

  /**
   * Creates new command that drives specified trajectory then attempts to locate the note with
   * vision and drives towards it with intake running until game piece is acquired.
   *
   * <p>This version has a fallback to drive to the note supplied location in case vision failed.
   */
  private Command driveThenIntakeWithFallback(
      String trajectoryName, Pose2d notePose, double timeoutSeconds) {
    return new DriveTrajectory(trajectories.get(trajectoryName), false, true, drive)
        .andThen(Commands.waitSeconds(0.06)) // note detection vision has a perceived lag of 60ms
        .andThen(
            Commands.either(
                    commands.driveToNote(),
                    commands.driveToPose(
                        FieldConstants.getFieldPose(notePose),
                        true,
                        shooter::isFrontBeamTrippedAndTimeout),
                    noteDetector::hasTrackedNote)
                .deadlineWith(commands.intake()))
        .withTimeout(timeoutSeconds);
  }

  private Command intakeWithFallback(Pose2d notePose, double timeoutSeconds) {
    return Commands.either(
            commands.driveToNote(),
            commands.driveToPose(
                FieldConstants.getFieldPose(notePose), true, shooter::isFrontBeamTrippedAndTimeout),
            noteDetector::hasTrackedNote)
        .deadlineWith(commands.intake())
        .withTimeout(timeoutSeconds);
  }

  /**
   * Repositions the robot slightly away from the stage pillar to allow the shot to go through,
   * since the pillar often prevents the robot to complete necessary rotation to target.
   */
  private Command driveToPodiumShot() {
    return commands
        .driveToPose(FieldConstants.getFieldPose(FieldConstants.podiumShot), true, null)
        .withTimeout(1.5);
  }

  private Command driveTrajectory(String trajectoryName, double timeoutSeconds) {
    return new DriveTrajectory(trajectories.get(trajectoryName), false, true, drive)
        .withTimeout(timeoutSeconds);
  }

  /** Creates a wrapper with timeout for safety during auto. */
  private Command scoreInSpeakerAuto() {
    return commands.scoreInSpeaker().withTimeout(4);
  }

  // private Command defaultAuto(String trajectory1, String trajectory2) {
  //   return Commands.sequence(
  //       scoreInSpeakerAuto(),
  //       driveAndIntake(trajectory1, 3, true),
  //       scoreInSpeakerAuto(),
  //       driveAndIntake(trajectory2, 4, false));
  // }

  // private Command defaultAuto(String trajectory) {
  //   return Commands.sequence(
  //       scoreInSpeakerAuto(), driveAndIntake(trajectory, 3, true), scoreInSpeakerAuto());
  // }

  // private Command defaultAuto(Supplier<Pose2d> pose, String trajectory) {
  //   return Commands.sequence(
  //       commands.setPose(pose),
  //       scoreInSpeakerAuto(),
  //       driveAndIntake(trajectory, 3, false),
  //       scoreInSpeakerAuto());
  // }

  // private Command defaultAuto(Supplier<Pose2d> pose, String... trajectories) {
  //   final var sequence = new Command[1 + trajectories.length * 2];
  //   sequence[0] = commands.setPose(pose);
  //   var offset = 1;
  //   for (var trajectory : trajectories) {
  //     sequence[offset] = scoreInSpeakerAuto();
  //     sequence[offset + 1] = driveAndIntake(trajectory, 4, false);
  //     offset += 2;
  //   }

  //   return Commands.sequence(sequence);
  // }

  // private Command waterlooAuto(Supplier<Pose2d> pose, String... trajectories) {
  //   return Commands.sequence(
  //       commands.setPose(pose),
  //       scoreInSpeakerAuto(),
  //       driveAndIntake(trajectories[0], 4, false),
  //       driveTrajectory(trajectories[1], 3, false),
  //       scoreInSpeakerAuto());
  // }

  /** Sets up our auto chooser and auto objects. */
  private void configureAutoRoutines() {
    // OLD AUTO ROUTINES -- DO NOT USE
    // STARTING POSITIONS ARE DIFFERENT, NO VISION, NO POSE FALLBACK
    // autoChooser.addOption(
    //     "Amp Auto",
    //     defaultAuto(FieldConstants.getFieldPose(FieldConstants.initialAmp), "InitAmpToNote"));
    // autoChooser.addOption("Center Auto", defaultAuto("InitCenterToNote"));
    // autoChooser.addOption("Load Auto", defaultAuto("InitLoadToNote"));
    // autoChooser.addOption(
    //     "Amp Auto 1",
    //     defaultAuto(
    //         FieldConstants.getFieldPose(FieldConstants.initialAmp), "InitAmpToNote",
    // "AmpToNote1"));
    // autoChooser.addOption(
    //     "Amp Auto 2",
    //     defaultAuto(
    //         FieldConstants.getFieldPose(FieldConstants.initialAmp), "InitAmpToNote",
    // "AmpToNote2"));
    // autoChooser.addOption(
    //     "Amp Auto Grand",
    //     defaultAuto(
    //         FieldConstants.getFieldPose(FieldConstants.initialAmp),
    //         "InitAmpToNote",
    //         "AmpToNote1",
    //         "Note1ToShoot1",
    //         "Shoot1ToNote2"));
    // autoChooser.addOption(
    //     "Wateroo Auto",
    //     waterlooAuto(
    //         FieldConstants.getFieldPose(FieldConstants.initialSource),
    //         "WaterlooPath1",
    //         "WaterlooPath2"));
    // autoChooser.addOption("Center Auto 1", defaultAuto("InitCenterToNote", "CenterToNote1"));
    // autoChooser.addOption("Center Auto 2", defaultAuto("InitCenterToNote", "CenterToNote2"));
    // autoChooser.addOption("Center Auto 3", defaultAuto("InitCenterToNote", "CenterToNote3"));
    // autoChooser.addOption("Center Auto 4", defaultAuto("InitCenterToNote", "CenterToNote4"));
    // autoChooser.addOption("Load Auto 5", defaultAuto("InitLoadToNote", "StageToNote5"));

    autoChooser.addOption(
        "Center - Three Notes",
        Commands.sequence(
            commands.setPose(FieldConstants.getFieldPose(FieldConstants.initialCenter)),
            scoreInSpeakerAuto(),
            driveThenIntakeWithFallback("C_N1", FieldConstants.note1, 4),
            scoreInSpeakerAuto(),
            driveThenIntakeWithFallback("N1_N2", FieldConstants.note2, 4),
            scoreInSpeakerAuto(),
            driveThenIntakeWithFallback("N2_N3", FieldConstants.note3, 4),
            driveToPodiumShot(),
            scoreInSpeakerAuto()));

    autoChooser.addOption(
        "Center - Three Notes",
        Commands.sequence(
            commands.setPose(FieldConstants.getFieldPose(FieldConstants.initialCenter)),
            scoreInSpeakerAuto(),
            driveThenIntakeWithFallback("C_N1", FieldConstants.note1, 4),
            scoreInSpeakerAuto(),
            driveThenIntakeWithFallback("N1_N2", FieldConstants.note2, 4),
            scoreInSpeakerAuto(),
            driveThenIntakeWithFallback("N2_N3", FieldConstants.note3, 4),
            driveToPodiumShot(),
            scoreInSpeakerAuto()));

    autoChooser.addOption(
        "Center - Two Notes",
        Commands.sequence(
            commands.setPose(FieldConstants.getFieldPose(FieldConstants.initialCenter)),
            scoreInSpeakerAuto(),
            intakeWithFallback(FieldConstants.note2, 4),
            scoreInSpeakerAuto(),
            driveThenIntakeWithFallback("N2_N3", FieldConstants.note3, 4),
            driveToPodiumShot(),
            scoreInSpeakerAuto()));

    autoChooser.addOption(
        "Center - N1 + CN2",
        Commands.sequence(
                commands.setPose(FieldConstants.getFieldPose(FieldConstants.initialCenter)),
                scoreInSpeakerAuto(),
                driveThenIntakeWithFallback("C_N1", FieldConstants.note1, 4),
                scoreInSpeakerAuto(),
                driveThenIntakeWithFallback("N1_CN2", FieldConstants.centralNote2, 4),
                driveTrajectory("CN2_S", 3),
                scoreInSpeakerAuto())
            .until(() -> RobotState.getInstance().isOverCenterLine()));

    autoChooser.addOption(
        "Center - N2 + CN3",
        Commands.sequence(
                commands.setPose(FieldConstants.getFieldPose(FieldConstants.initialCenter)),
                scoreInSpeakerAuto(),
                intakeWithFallback(FieldConstants.note2, 4),
                scoreInSpeakerAuto(),
                driveThenIntakeWithFallback("N2_CN3", FieldConstants.centralNote2, 4),
                driveTrajectory("CN3_S", 3),
                scoreInSpeakerAuto())
            .until(() -> RobotState.getInstance().isOverCenterLine()));

    autoChooser.addOption(
        "Amp - N1 + CN2",
        Commands.sequence(
                commands.setPose(FieldConstants.getFieldPose(FieldConstants.initialAmp)),
                scoreInSpeakerAuto(),
                driveThenIntakeWithFallback("A_N1", FieldConstants.note1, 4),
                scoreInSpeakerAuto(),
                driveThenIntakeWithFallback("N1_CN2", FieldConstants.centralNote2, 4),
                driveTrajectory("CN2_S", 3),
                scoreInSpeakerAuto())
            .until(() -> RobotState.getInstance().isOverCenterLine()));

    autoChooser.addOption(
        "Load - CN4",
        Commands.sequence(
                commands.setPose(FieldConstants.getFieldPose(FieldConstants.initialLoad)),
                scoreInSpeakerAuto(),
                driveThenIntakeWithFallback("L_CN4", FieldConstants.centralNote4, 5),
                driveTrajectory("CN4_S", 3),
                scoreInSpeakerAuto())
            .until(() -> RobotState.getInstance().isOverCenterLine()));

    autoChooser.addOption(
        "Load - N3 -OPDAVE",
        Commands.sequence(
                commands.setPose(FieldConstants.getFieldPose(FieldConstants.initialLoad)),
                scoreInSpeakerAuto(),
                driveThenIntakeWithFallback("L_N3", FieldConstants.note1, 3),
                driveTrajectory("N3_L", 3),
                scoreInSpeakerAuto())
            .until(() -> RobotState.getInstance().isOverCenterLine()));

    autoChooser.addOption(
        "Load - CN5",
        Commands.sequence(
                commands.setPose(FieldConstants.getFieldPose(FieldConstants.initialLoad)),
                scoreInSpeakerAuto(),
                driveThenIntakeWithFallback("L_CN5", FieldConstants.centralNote5, 5),
                driveTrajectory("CN5_S", 3),
                scoreInSpeakerAuto())
            .until(() -> RobotState.getInstance().isOverCenterLine()));

    autoChooser.addOption(
        "Load - Wreck",
        Commands.sequence(
                commands.setPose(FieldConstants.getFieldPose(FieldConstants.initialLoad)),
                scoreInSpeakerAuto(),
                driveTrajectory("L_WRECK", 7))
            .until(() -> RobotState.getInstance().isOverCenterLine()));

    autoChooser.addOption(
        "Wheel Diameter Characterization", commands.wheelDiameterCharacterization());

    autoChooser.addOption(
        "Drive FF Characterization",
        new FeedForwardCharacterization(
            drive, drive::runCharacterization, drive::getCharacterizationVelocity));

    // trajectories.forEach(
    //     (k, v) -> autoChooser.addOption("Path " + k, new DriveTrajectory(v, true, true, drive)));

    autoChooser.addOption("Just Shoot", commands.scoreInSpeaker());
    autoChooser.addOption("Do Nothing", Commands.none());
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureBindings() {
    drive.setDefaultCommand(
        commands.driveTeleop(
            () -> -controllerDriver.getLeftX(),
            () -> -controllerDriver.getLeftY(),
            () -> -controllerDriver.getRightX()));

    led.setDefaultCommand(commands.led(LedMode.DEFAULT));

    // controllerDriver.x().whileTrue(Commands.run(drive::stopWithX).withName("StopWithX"));
    controllerDriver
        .start()
        .onTrue(
            Commands.runOnce(
                    () -> {
                      drive.setPose(new Pose2d(new Translation2d(), new Rotation2d()));
                      drive.reset();
                    })
                .ignoringDisable(true)
                .withName("ResetOdometry"));

    // // orientation locks
    // controllerDriver
    //     .leftBumper()
    //     .onTrue(
    //         Commands.either(
    //             Commands.runOnce(() -> drive.toggleOrientationLock(Math.toRadians(90))),
    //             Commands.runOnce(() -> drive.toggleOrientationLock(Math.toRadians(-90))),
    //             () -> DriverStation.getAlliance().get() == Alliance.Blue));

    // controllerDriver
    //     .rightBumper()
    //     .onTrue(
    //         Commands.either(
    //             Commands.runOnce(() -> drive.toggleOrientationLock(Math.toRadians(-45))),
    //             Commands.runOnce(() -> drive.toggleOrientationLock(Math.toRadians(45))),
    //             () -> DriverStation.getAlliance().get() == Alliance.Blue));

    controllerDriver.rightBumper().onTrue(commands.pass());

    // intake
    // LT only -- regular intake
    // LT + X -- intake with vision
    // X only -- pass
    controllerDriver.leftTrigger().and(controllerDriver.x().negate()).whileTrue(commands.intake());
    controllerDriver
        .leftTrigger()
        .onFalse(commands.reindex().onlyIf(() -> shooter.isFrontBeamTripped()).withTimeout(2));

    controllerDriver
        .leftTrigger()
        .and(controllerDriver.x())
        .whileTrue(commands.autoIntakeViaVision());

    controllerDriver.x().and(controllerDriver.leftTrigger().negate()).onTrue(commands.pass());

    // eject
    controllerDriver.y().whileTrue(commands.eject());

    // speaker shot
    controllerDriver
        .rightTrigger()
        .whileTrue(commands.scoreInSpeaker().alongWith(commands.led(LedMode.SPEAKER)));

    // subwoofer shot
    controllerDriver
        .b()
        .whileTrue(
            commands
                .moveArmToPosition(50)
                .alongWith(commands.shoot(shooterSubwooferSpeed.get(), pivot::isAtGoal)))
        .onFalse(commands.moveArmToPosition(20, 2));

    // amp shot
    // controllerDriver
    //     .a()
    //     .onTrue(
    //         Commands.parallel(
    //                 commands.moveArmToPosition(107, 5),
    //                 commands.shoot(900, pivot::isAtGoal).until(() -> !shooter.hasGamePiece()))
    //             .andThen(commands.moveArmToPosition(20, 3))
    //             .deadlineWith(commands.led(LedMode.AMP)));

    controllerDriver
        .a()
        .onTrue(
            Commands.parallel(
                    Commands.runOnce(() -> shooter.setSpeed(800), shooter)
                        .alongWith(commands.moveArmToPosition(90, 3)))
                .andThen(
                    Commands.runOnce(() -> shooter.feedForward(), shooter)
                        .alongWith(commands.moveArmToPosition(107)))
                .andThen(
                    commands
                        .moveArmToPosition(20)
                        .alongWith(Commands.runOnce(() -> shooter.stop(), shooter)))
                .deadlineWith(commands.led((LedMode.AMP))));

    // move to idle
    controllerDriver
        .povDown()
        .onTrue(
            Commands.parallel(
                commands.moveArmToPosition(20), Commands.runOnce(shooter::stop, shooter)));

    controllerDriver.povLeft().onTrue(commands.reindex().withTimeout(2));

    // climb commands
    controllerDriver.povUp().onTrue(commands.raiseHooks().withTimeout(5));
    controllerDriver.back().onTrue(commands.retractHooks().withTimeout(2));
  }

  /** Use this method to put commands on the SmartDashboard. */
  private void configureSmartDashboard() {
    SmartDashboard.putData("Climb/Home", commands.homeHooks());
    SmartDashboard.putData("Climb/Release Ratchet", commands.releaseRatchet());
    SmartDashboard.putData("Climb/Engage Ratchet", commands.engageRatchet());
    SmartDashboard.putData("Trap/Shoot", commands.trapShot());
    SmartDashboard.putNumber("Trap/Angle", 59);
    SmartDashboard.putNumber("Trap/RPM", 2200);
  }

  /**
   * Use this method to configure haptic feedback to the driver / operator when certain game
   * milestones have been reached, e.g. end game.
   */
  private void configureTriggers() {
    Function<Double, Command> controllerRumbleCommandFactory =
        time ->
            Commands.sequence(
                Commands.runOnce(
                    () -> controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 1.0)),
                Commands.waitSeconds(time),
                Commands.runOnce(
                    () -> controllerDriver.getHID().setRumble(RumbleType.kBothRumble, 0.0)));

    new Trigger(() -> shooter.isFrontBeamTripped())
        .onTrue(
            commands
                .led(LedMode.ACQUIRED_NOTE)
                .withTimeout(1.0)
                .alongWith(controllerRumbleCommandFactory.apply(0.3)));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert1.get()))
        .onTrue(controllerRumbleCommandFactory.apply(0.5));

    new Trigger(
            () ->
                DriverStation.isTeleopEnabled()
                    && DriverStation.getMatchTime() > 0
                    && DriverStation.getMatchTime() <= Math.round(endgameAlert2.get()))
        .onTrue(
            Commands.sequence(
                controllerRumbleCommandFactory.apply(0.2),
                Commands.waitSeconds(0.1),
                controllerRumbleCommandFactory.apply(0.2),
                Commands.waitSeconds(0.1),
                controllerRumbleCommandFactory.apply(0.2)));
  }

  public Command getAutonomousCommand() {
    return autoChooser.get();
  }

  public void onDisable() {
    CommandScheduler.getInstance().schedule(commands.disablePivot());
    CommandScheduler.getInstance().schedule(commandLedDisabled);
  }

  public void setLoggingIndicator(boolean logging) {
    if (logging) {
      CommandScheduler.getInstance().schedule(commandLedLogging);
    } else {
      CommandScheduler.getInstance().schedule(commandLedDisabled);
    }
  }

  public void updateVisionTracking() {
    final var tracking = aprilTagDetector.areTagsTracked(7, 4, 2);
    RobotState.getInstance().setTrackingTags(tracking);
    led.setTracking(tracking);
    led.setTrackingBoth(aprilTagDetector.areTagsTracked(3, 8));
  }

  public void onAutoInit() {
    final var timestamp = Timer.getFPGATimestamp();
    RobotState.getInstance().autoStartedAt = timestamp;
    System.out.println("Started autonomous at: " + timestamp);

    // start shooter with flywheel in break mode to use in autonomous
    // as a fix for notes popping right through the shooter during intake
    shooter.setFlywheelBrakeMode(true);
  }

  public void onTeleopInit() {
    shooter.setFlywheelBrakeMode(false);
  }
}
