package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import choreo.Choreo.TrajectoryLogger;
import choreo.auto.AutoFactory;
import choreo.trajectory.SwerveSample;
import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;
import frc.robot.lib.SysIdSwerveTranslationTorqueCurrent;
import frc.robot.lib.Tunable;
import java.util.Optional;
import java.util.function.Supplier;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements subsystem so it can easily
 * be used in command-based projects.
 */
public class CommandSwerveDrivetrain extends TunerSwerveDrivetrain implements Subsystem {
  private static final double kSimLoopPeriod = 0.005; // 5 ms
  private Notifier simNotifier = null;
  private double lastSimTime;

  // kSpeedAt12Volts desired top speed
  private static final double maxSpeedMetersPerSecond = DriveConstants.maxSpeed.in(MetersPerSecond);
  private static final double maxAngularRateRadiansPerSecond =
      DriveConstants.maxAngularRate.in(RadiansPerSecond);

  private static double kControllerDeadband = 0; // e.g. 0.05 for 5%
  private static double kAccelerationRateLimit = 1; // 100% in 1 second
  private static double kDecelarationRateLimit = 5; // 100% in 200 ms

  // teleop slew rate limiter for linear speed
  private final SlewRateLimiter teleopVelocityLimiter =
      new SlewRateLimiter(kAccelerationRateLimit, -kDecelarationRateLimit, 0);

  // teleop slew rate limiter for rotational speed (omega)
  private final SlewRateLimiter teleopRotationLimiter =
      new SlewRateLimiter(kAccelerationRateLimit, -kDecelarationRateLimit, 0);

  private static final ChassisSpeeds kZeroSpeeds = new ChassisSpeeds();

  // Blue alliance sees forward as 0 degrees (toward red alliance wall)
  private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;

  // Red alliance sees forward as 180 degrees (toward blue alliance wall)
  private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

  // keep track if we've ever applied the operator perspective before or not
  private boolean hasAppliedOperatorPerspective = false;

  // swerve request to apply for brake position (X configuration)
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

  // swerve request for teleop driving
  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          .withDeadband(maxSpeedMetersPerSecond * kControllerDeadband)
          .withRotationalDeadband(maxAngularRateRadiansPerSecond * kControllerDeadband)
          .withDriveRequestType(DriveRequestType.Velocity);

  // swerve request for teleop driving with orientation lock
  private final SwerveRequest.FieldCentricFacingAngle driveFacingAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDeadband(maxSpeedMetersPerSecond * kControllerDeadband)
          .withRotationalDeadband(maxAngularRateRadiansPerSecond * kControllerDeadband)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withHeadingPID(
              DriveConstants.PathFollowing.rotationKp,
              DriveConstants.PathFollowing.rotationKi,
              DriveConstants.PathFollowing.rotationKd);

  // swerve request for auto driving
  private final SwerveRequest.RobotCentric driveRobot =
      new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.Velocity);

  // swerve request for auto driving with orientation lock
  private final SwerveRequest.RobotCentricFacingAngle driveRobotFacingAngle =
      new SwerveRequest.RobotCentricFacingAngle()
          .withForwardPerspective(ForwardPerspectiveValue.BlueAlliance)
          .withDriveRequestType(DriveRequestType.Velocity)
          .withHeadingPID(
              DriveConstants.PathFollowing.rotationKp,
              DriveConstants.PathFollowing.rotationKi,
              DriveConstants.PathFollowing.rotationKd);

  // swerve request for pointing wheels forward
  private final SwerveRequest.PointWheelsAt wheelsForward =
      new SwerveRequest.PointWheelsAt()
          .withModuleDirection(Rotation2d.kZero)
          .withDriveRequestType(DriveRequestType.Velocity);

  // swerve request to apply during field-centric path following
  private final SwerveRequest.ApplyFieldSpeeds pathApplyFieldSpeeds =
      new SwerveRequest.ApplyFieldSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  // swerve request to apply during robot-centric path following
  private final SwerveRequest.ApplyRobotSpeeds pathApplyRobotSpeeds =
      new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

  // PID controllers used with Choreo path following
  private final PIDController pathXController =
      new PIDController(
          DriveConstants.PathFollowing.translationKp,
          DriveConstants.PathFollowing.translationKi,
          DriveConstants.PathFollowing.translationKd,
          Constants.cyclePeriodSeconds);

  private final PIDController pathYController =
      new PIDController(
          DriveConstants.PathFollowing.translationKp,
          DriveConstants.PathFollowing.translationKi,
          DriveConstants.PathFollowing.translationKd,
          Constants.cyclePeriodSeconds);

  private final PIDController pathThetaController =
      new PIDController(
          DriveConstants.PathFollowing.rotationKp,
          DriveConstants.PathFollowing.rotationKi,
          DriveConstants.PathFollowing.rotationKd,
          Constants.cyclePeriodSeconds);

  // swerve requests to apply during SysId characterization
  private final SwerveRequest.SysIdSwerveTranslation translationVoltageCharacterization =
      new SwerveRequest.SysIdSwerveTranslation();
  private final SysIdSwerveTranslationTorqueCurrent translationCurrentCharacterization =
      new SysIdSwerveTranslationTorqueCurrent();
  private final SwerveRequest.SysIdSwerveSteerGains steerCharacterization =
      new SwerveRequest.SysIdSwerveSteerGains();
  private final SwerveRequest.SysIdSwerveRotation rotationCharacterization =
      new SwerveRequest.SysIdSwerveRotation();

  // SysId routine for characterizing translation (used to find PID gains for the drive motors)
  private final SysIdRoutine sysIdRoutineTranslationVoltage =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // default ramp rate (1V/s)
              Volts.of(4), // reduce dynamic step voltage to 4V to prevent brownout
              null, // default timeout (10s)
              // log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslationVoltage_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> setControl(translationVoltageCharacterization.withVolts(output)),
              null,
              this));

  // SysId routine for characterizing translation (used to find PID gains for the drive motors)
  // [!] SysId expects Volts, treat Volts as Amps for characterization purposes
  private final SysIdRoutine sysIdRoutineTranslationCurrent =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(10).per(Second), // ramp of 10A/s
              Volts.of(20), // dynamic step of 20A
              null, // default timeout (10s)
              // log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdTranslationCurrent_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output ->
                  setControl(
                      translationCurrentCharacterization.withTorqueCurrent(output.in(Volts))),
              null,
              this));

  // SysId routine for characterizing steer (used to find PID gains for the steer motors)
  private final SysIdRoutine sysIdRoutineSteer =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              null, // default ramp rate (1V/s)
              Volts.of(7), // dynamic voltage of 7V
              null, // default timeout (10s)
              // log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
          new SysIdRoutine.Mechanism(
              volts -> setControl(steerCharacterization.withVolts(volts)), null, this));

  // SysId routine for characterizing rotation (used to find PID gains for the
  // FieldCentricFacingAngle HeadingController)
  //
  // See docs of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId
  private final SysIdRoutine sysIdRoutineRotation =
      new SysIdRoutine(
          new SysIdRoutine.Config(
              Volts.of(Math.PI / 6).per(Second), // in radians per second²
              Volts.of(Math.PI), // in radians per second
              null, // default timeout (10 s)
              // log state with SignalLogger class
              state -> SignalLogger.writeString("SysIdRotation_State", state.toString())),
          new SysIdRoutine.Mechanism(
              output -> {
                // output is actually radians per second, but SysId only supports "volts"
                setControl(rotationCharacterization.withRotationalRate(output.in(Volts)));
                // also log the requested output for SysId
                SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
              },
              null,
              this));

  // SysId routine to use
  private SysIdRoutine sysIdRoutineToApply = sysIdRoutineTranslationVoltage;

  // angle for orientation lock
  private Optional<Rotation2d> orientationLock = Optional.empty();

  private void init() {
    pathThetaController.enableContinuousInput(-Math.PI, Math.PI);

    if (Utils.isSimulation()) {
      startSimThread();
    }

    pathXController.setTolerance(Constants.DriveConstants.PathFollowing.kTranslationTolerance);
    pathYController.setTolerance(Constants.DriveConstants.PathFollowing.kTranslationTolerance);
    pathThetaController.setTolerance(Constants.DriveConstants.PathFollowing.kRotationTolerance);

    Tunable.bind("Drive/PathX", pathXController);
    Tunable.bind("Drive/PathY", pathYController);
    Tunable.bind("Drive/PathTheta", pathThetaController);
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, modules);
    init();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(drivetrainConstants, odometryUpdateFrequency, modules);
    init();
  }

  /**
   * Constructs a CTRE SwerveDrivetrain using the specified constants.
   *
   * <p>This constructs the underlying hardware devices, so users should not construct the devices
   * themselves. If they need the devices, they can access them through getters in the classes.
   *
   * @param drivetrainConstants Drivetrain-wide constants for the swerve drive
   * @param odometryUpdateFrequency The frequency to run the odometry loop. If unspecified or set to
   *     0 Hz, this is 250 Hz on CAN FD, and 100 Hz on CAN 2.0.
   * @param odometryStandardDeviation The standard deviation for odometry calculation in the form
   *     [x, y, theta]ᵀ, with units in meters and radians
   * @param visionStandardDeviation The standard deviation for vision calculation in the form [x, y,
   *     theta]ᵀ, with units in meters and radians
   * @param modules Constants for each specific module
   */
  public CommandSwerveDrivetrain(
      SwerveDrivetrainConstants drivetrainConstants,
      double odometryUpdateFrequency,
      Matrix<N3, N1> odometryStandardDeviation,
      Matrix<N3, N1> visionStandardDeviation,
      SwerveModuleConstants<?, ?, ?>... modules) {
    super(
        drivetrainConstants,
        odometryUpdateFrequency,
        odometryStandardDeviation,
        visionStandardDeviation,
        modules);
    init();
  }

  /**
   * Returns current robot's pose.
   *
   * @return The current pose of the robot.
   */
  public Pose2d getPose() {
    return getState().Pose;
  }

  /**
   * Returns current robot-centric velocity.
   *
   * @return The current robot-centric chassis speeds.
   */
  public ChassisSpeeds getRobotSpeeds() {
    return getState().Speeds;
  }

  /**
   * Returns roll angle.
   *
   * @return The IMU reported roll angle.
   */
  public Angle getRoll() {
    return getPigeon2().getRoll().getValue();
  }

  /**
   * Returns pitch angle.
   *
   * @return The IMU reported pitch angle.
   */
  public Angle getPitch() {
    return getPigeon2().getPitch().getValue();
  }

  /**
   * Returns current field-centric velocity.
   *
   * @return The current field-centric chassis speeds.
   */
  public ChassisSpeeds getFieldSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(getRobotSpeeds(), getPose().getRotation());
  }

  /**
   * Returns observed robot's pose at the specified timestamp.
   *
   * @param timestamp FPGA timestamp in microseconds.
   * @return The observed pose of the robot or Optional.empty().
   */
  public Optional<Pose2d> getPoseAt(long timestamp) {
    // uses SwerveDrivetrain internal buffer, requires timestamp conversion as per:
    //
    // "Note that you must use a timestamp with an epoch since system startup (i.e.,
    // the epoch of this timestamp is the same epoch as Utils.getCurrentTimeSeconds()).
    // This means that you should use Utils.getCurrentTimeSeconds() as your time source
    // in this case. An FPGA timestamp can be converted to the correct timebase using
    // Utils.fpgaToCurrentTime(double)."
    return samplePoseAt(Utils.fpgaToCurrentTime(timestamp / 1000000.0));
  }

  /**
   * Creates a new auto factory for this drivetrain.
   *
   * @return AutoFactory for this drivetrain
   */
  public AutoFactory createAutoFactory() {
    return createAutoFactory((sample, isStart) -> {});
  }

  /**
   * Creates a new auto factory for this drivetrain with the given trajectory logger.
   *
   * @param trajectoryLogger Logger for the trajectory
   * @return AutoFactory for this drivetrain
   */
  public AutoFactory createAutoFactory(TrajectoryLogger<SwerveSample> trajectoryLogger) {
    return new AutoFactory(
        this::getPose, this::resetPose, this::driveVia, true, this, trajectoryLogger);
  }

  /**
   * Returns a command that applies the specified control request to this swerve drivetrain.
   *
   * @param request Function returning the request to apply
   * @return Command to run
   */
  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> setControl(requestSupplier.get()));
  }

  /**
   * Returns a command that applies specified teleop inputs to this drivetrain.
   *
   * @param vxSupplier X velocity supplier (m/s)
   * @param vySupplier Y velocity supplier (m/s)
   * @param omegaSupplier Omega supplier (rad/s)
   * @return Command to run
   */
  public Command driveTeleop(
      Supplier<Double> vxSupplier, Supplier<Double> vySupplier, Supplier<Double> omegaSupplier) {
    return run(
        () -> {
          // do not accept teleop input during autonomous:
          // this prevents the situation when a rogue (broken) controller
          // somehow sends inputs, e.g. due to driver or hardware malfunction
          if (DriverStation.isAutonomous()) {
            return;
          }

          final var vx = vxSupplier.get();
          final var vy = vySupplier.get();
          final var w = omegaSupplier.get();

          // apply velocity rate limiter
          final var heading = Math.atan2(vx, vy);
          final var v = MathUtil.clamp(teleopVelocityLimiter.calculate(Math.hypot(vx, vy)), -1, 1);

          driveAtFieldRelative(
              v * Math.sin(heading) * maxSpeedMetersPerSecond,
              v * Math.cos(heading) * maxSpeedMetersPerSecond,
              MathUtil.clamp(Math.signum(w) * teleopRotationLimiter.calculate(Math.abs(w)), -1, 1)
                  * maxAngularRateRadiansPerSecond);
        });
  }

  /**
   * Returns a command that applies break position (X).
   *
   * @return Command to run
   */
  public Command brake() {
    return run(() -> setControl(brake));
  }

  /**
   * Returns a command that sets wheels position forward.
   *
   * @return Command to run
   */
  public Command wheelsForward() {
    return run(() -> setControl(wheelsForward));
  }

  /**
   * Follows the given field-centric (Choreo) path sample with PID.
   *
   * @param sample Sample along the path to follow
   */
  public void driveVia(SwerveSample sample) {
    final var pose = getPose();
    final var targetSpeeds = sample.getChassisSpeeds();

    targetSpeeds.vxMetersPerSecond += pathXController.calculate(pose.getX(), sample.x);
    targetSpeeds.vyMetersPerSecond += pathYController.calculate(pose.getY(), sample.y);
    targetSpeeds.omegaRadiansPerSecond +=
        pathThetaController.calculate(pose.getRotation().getRadians(), sample.heading);

    setControl(
        pathApplyFieldSpeeds
            .withSpeeds(targetSpeeds)
            .withWheelForceFeedforwardsX(sample.moduleForcesX())
            .withWheelForceFeedforwardsY(sample.moduleForcesY()));
  }

  /**
   * Drives with the specified robot-centric speeds.
   *
   * @param speeds robot-centric chassis speeds
   */
  public void driveAtRobotRelative(ChassisSpeeds speeds) {
    setControl(pathApplyRobotSpeeds.withSpeeds(speeds));
  }

  /**
   * Drives with the specified field-relative input velocities and omega. Honors orientation lock.
   *
   * <p>Use this version for teleop control.
   */
  public void driveAtFieldRelative(double vx, double vy, double omega) {
    final var request =
        orientationLock.isPresent()
            ? driveFacingAngle
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withTargetDirection(orientationLock.get())
            : drive.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega);

    setControl(request);
  }

  /**
   * Drives with the specified robot-relative input velocities and omega. Honors orientation lock.
   * Uses field-perspective aka blue alliance origin.
   *
   * <p>Use this version for autonomous control.
   */
  public void driveAtRobotRelative(double vx, double vy, double omega) {
    final var request =
        orientationLock.isPresent()
            ? driveRobotFacingAngle
                .withVelocityX(vx)
                .withVelocityY(vy)
                .withTargetDirection(orientationLock.get())
            : driveRobot.withVelocityX(vx).withVelocityY(vy).withRotationalRate(omega);

    setControl(request);
  }

  /** Stops the robot by setting speeds to zero. */
  public void stop() {
    driveAtRobotRelative(kZeroSpeeds);
  }

  /** Determines whether robot is at the specified pose. */
  public boolean isAtPose(Pose2d pose) {
    final var robotPose = getPose();

    final var d = pose.getTranslation().getDistance(robotPose.getTranslation());
    final var r = Math.abs(pose.getRotation().getRadians() - robotPose.getRotation().getRadians());

    return d <= Constants.DriveConstants.PoseChasing.kTranslationTolerance
        && r <= Constants.DriveConstants.PoseChasing.kRotationTolerance;
  }

  /**
   * Sets the orientation lock.
   *
   * @param value Operator-perspective orientation to maintain.
   */
  public void setOrientationLock(Rotation2d value) {
    orientationLock = Optional.of(value);
  }

  /** Removes the orientation lock. */
  public void clearOrientationLock() {
    orientationLock = Optional.empty();
  }

  /** Determines whether orientation lock is enabled. */
  public boolean hasOrientationLock() {
    return orientationLock.isPresent();
  }

  /**
   * Runs the SysId Quasistatic test in the given direction for the routine specified by {@link
   * #sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Quasistatic test
   * @return Command to run
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysIdRoutineToApply.quasistatic(direction);
  }

  /**
   * Runs the SysId Dynamic test in the given direction for the routine specified by {@link
   * #sysIdRoutineToApply}.
   *
   * @param direction Direction of the SysId Dynamic test
   * @return Command to run
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysIdRoutineToApply.dynamic(direction);
  }

  @Override
  public void periodic() {
    // periodically try to apply the operator perspective: if we haven't applied the operator
    // perspective before, then we should apply it regardless of DS state.
    // This allows us to correct the perspective in case the robot code restarts mid-match.
    // Otherwise, only check and apply the operator perspective if the DS is disabled.
    // This ensures driving behavior doesn't change until an explicit disable event occurs during
    // testing.
    if (!hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
      DriverStation.getAlliance()
          .ifPresent(
              allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation);
                hasAppliedOperatorPerspective = true;
              });
    }
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    // run simulation at a faster rate so PID gains behave more reasonably
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              // use the measured time delta, get battery voltage from WPILib
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });

    simNotifier.startPeriodic(kSimLoopPeriod);
  }
}
