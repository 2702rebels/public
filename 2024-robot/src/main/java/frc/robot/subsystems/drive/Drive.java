package frc.robot.subsystems.drive;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PathPlannerLogging;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveWheelPositions;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotState;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Misc;
import java.util.OptionalDouble;
import java.util.function.Function;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
  private final GyroIO gyroIO;
  private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
  private final Module[] modules = new Module[4]; // FL, FR, RL, RR

  private final DriveOdometryTracker odometryTracker;
  private final DriveOdometryInputsAutoLogged odometryInputs = new DriveOdometryInputsAutoLogged();

  // maintain a pre-allocated buffer to populate with recently currently sampled
  // set of module positions on each iteration of odometry update, this cuts down
  // on the number of array allocations required to construct odometry readings
  private final SwerveDriveWheelPositions[] sampledWheelPositions;

  private final RobotState robotState;
  private Twist2d fieldVelocity = new Twist2d();

  private final ChassisSpeeds ZeroChassisSpeeds = new ChassisSpeeds();
  private final SwerveModuleState[] stopXstates =
      new SwerveModuleState[] {
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(-45)),
        new SwerveModuleState(0, Rotation2d.fromDegrees(45)),
      };

  // measured module states
  private final SwerveModuleState[] measuredStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
      };

  // deltas between commanded setpoints and measured module states
  private final SwerveModuleState[] deltaStates =
      new SwerveModuleState[] {
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
        new SwerveModuleState(),
      };

  private static final SwerveModuleState[] emptyStates = new SwerveModuleState[] {};

  private static final LoggedTunableNumber orientationP =
      new LoggedTunableNumber("Drive/Orientation/P", 0.3);

  private static final LoggedTunableNumber steeringP =
      new LoggedTunableNumber("Drive/Steering/P", 0.3);

  // orientation to maintain while lock is in place
  private double orientationLockAngleRad;
  private boolean orientationLockEnabled;
  private final PIDController orientationController;
  private final PIDController steeringController;

  private OptionalDouble steeringTurnAngleRadians = OptionalDouble.empty();

  private boolean stopX;

  public Drive(GyroIO gyroIO, Function<Integer, ModuleIO> moduleBuilder) {
    // maximum number of attainable samples depends on the odometry frequency
    // and the period of the robot main cycle (plus one for the boundary case)
    final var samplesMax =
        (int) Math.ceil(Constants.cyclePeriodSeconds * Constants.odometryFrequency) + 1;

    sampledWheelPositions = new SwerveDriveWheelPositions[samplesMax];
    for (var i = 0; i < sampledWheelPositions.length; ++i) {
      sampledWheelPositions[i] =
          new SwerveDriveWheelPositions(
              new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
              });
    }

    // initialize swerve modules
    for (var i = 0; i < modules.length; ++i) {
      modules[i] = new Module(moduleBuilder.apply(i), i, DriveConstants.moduleAngularOffsets[i]);
    }

    this.gyroIO = gyroIO;
    this.robotState = frc.robot.RobotState.getInstance();
    this.odometryTracker = DriveOdometryTracker.getInstance();

    orientationController = new PIDController(orientationP.get(), 0, 0);
    orientationController.enableContinuousInput(-Math.PI, Math.PI);

    steeringController = new PIDController(steeringP.get(), 0, 0);
    steeringController.enableContinuousInput(-Math.PI, Math.PI);
    steeringController.setSetpoint(0);

    // configure AutoBuilder for PathPlanner
    AutoBuilder.configureHolonomic(
        this::getPose,
        this::setPose,
        () -> DriveConstants.kinematics.toChassisSpeeds(getModuleStates()),
        this::runVelocity,
        new HolonomicPathFollowerConfig(
            DriveConstants.drivetrainConfig.maxLinearSpeed(),
            DriveConstants.drivetrainConfig.driveBaseRadius(),
            new ReplanningConfig()),
        () ->
            DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get() == Alliance.Red,
        this);

    Pathfinding.setPathfinder(new LocalADStarAK());
    PathPlannerLogging.setLogActivePathCallback(
        (activePath) -> {
          Logger.recordOutput(
              "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
        });

    PathPlannerLogging.setLogTargetPoseCallback(
        (targetPose) -> {
          Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
        });
  }

  /**
   * Updates loggable inputs from IO modules impacting odometry. Must be invoked under
   * DriveOdometryTracker lock.
   */
  private void updateOdometryInputs() {
    gyroIO.updateInputs(gyroInputs);
    Logger.processInputs("Drive/Gyro", gyroInputs);

    var index = 0;
    for (var module : modules) {
      module.updateInputs();

      final var moduleState = module.getState();
      measuredStates[index].angle = moduleState.angle;
      measuredStates[index].speedMetersPerSecond = moduleState.speedMetersPerSecond;
      index++;
    }

    // update field velocity
    final var chassisSpeeds = DriveConstants.kinematics.toChassisSpeeds(measuredStates);
    final var linearFieldVelocity =
        new Translation2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond)
            .rotateBy(getRotation());

    fieldVelocity =
        new Twist2d(
            linearFieldVelocity.getX(),
            linearFieldVelocity.getY(),
            gyroInputs.connected
                ? gyroInputs.yawVelocityRadPerSec
                : chassisSpeeds.omegaRadiansPerSecond);
  }

  /** Updates robot state based from odometry observations. */
  private void updateOdometry() {
    odometryTracker.updateInputs(odometryInputs, this::updateOdometryInputs);
    Logger.processInputs("Drive/Odometry", odometryInputs);

    final var timestamps = odometryInputs.timestamps;
    final var yawPositions = gyroInputs.odometryYawPositions;

    // modules are sampled atomically with the timestamps,
    // we should have the same number of sampled entries for
    // each module as the number of collected timestamps
    final var count = Math.min(timestamps.length, sampledWheelPositions.length);
    for (var i = 0; i < modules.length; ++i) {
      final var sampledPositions = modules[i].getSampledPositionsMeters();
      final var sampledAngles = modules[i].getSampledAngles();
      for (var t = 0; t < count; ++t) {
        final var position = sampledWheelPositions[t].positions[i];
        position.distanceMeters = sampledPositions[t];
        position.angle = sampledAngles[t];
      }
    }

    for (var t = 0; t < count; ++t) {
      robotState.addOdometryObservation(
          sampledWheelPositions[t], gyroInputs.connected ? yawPositions[t] : null, timestamps[t]);
    }
  }

  /**
   * Calculates angular velocities for individual modules using second-order kinematics equations
   * under the assumption of no acceleration.
   *
   * <p>Note that there is no need for complex matrix/vector multiplication machinery, since we are
   * assuming constant chassis velocities, i.e. robot's linear and angular accelerations are
   * considered to be zero. As such we are reduced to a situation where module acceleration is just
   * a centripetal acceleration.
   */
  private static double[] calculateAngularVelocity(
      ChassisSpeeds chassisSpeeds, SwerveModuleState[] moduleStates) {
    final var angularVelocities = new double[moduleStates.length];
    final var omegaSq = Math.pow(chassisSpeeds.omegaRadiansPerSecond, 2);

    for (var i = 0; i < moduleStates.length; ++i) {
      final var rx = DriveConstants.moduleTranslations[i].getX();
      final var ry = DriveConstants.moduleTranslations[i].getY();
      final var speed = moduleStates[i].speedMetersPerSecond;
      final var angle = moduleStates[i].angle;

      angularVelocities[i] =
          (omegaSq / speed) * (angle.getSin() * rx - angle.getCos() * ry)
              - chassisSpeeds.omegaRadiansPerSecond;
    }

    return angularVelocities;
  }

  /** Computes delta between setpoint and current state to be used for telemetry and debugging. */
  private void updateModuleDelta(int index, SwerveModuleState setpointState) {
    final var setpointAngle = Misc.clampAngle(setpointState.angle.getRadians());
    final var measuredAngle = Misc.clampAngle(measuredStates[index].angle.getRadians());
    final var setpointSpeed = setpointState.speedMetersPerSecond;
    final var measuredSpeed = measuredStates[index].speedMetersPerSecond;

    var deltaAngle = setpointAngle - measuredAngle;
    var deltaSpeed = setpointSpeed - measuredSpeed;

    if (Math.abs(deltaAngle) > Math.PI * 0.5) {
      deltaAngle = Misc.clampAngle(setpointAngle - measuredAngle + Math.PI);
      deltaSpeed = -setpointSpeed - measuredSpeed;
    }

    deltaStates[index].speedMetersPerSecond = Math.abs(deltaSpeed);
    deltaStates[index].angle = Rotation2d.fromRadians(deltaAngle);
  }

  public void periodic() {
    final var callerId = hashCode();

    // update controllers if tunable numbers have changed
    if (orientationP.hasChanged(callerId)) {
      orientationController.setP(orientationP.get());
    }

    if (steeringP.hasChanged(callerId)) {
      steeringController.setP(orientationP.get());
    }

    for (var module : modules) {
      module.periodic();
    }

    updateOdometry();

    // stop moving when disabled
    if (DriverStation.isDisabled()) {
      for (var module : modules) {
        module.stop();
      }

      // log empty setpoint states when disabled
      Logger.recordOutput("Drive/SwerveStates/Setpoints", emptyStates);
      Logger.recordOutput("Drive/SwerveStates/Deltas", emptyStates);
    }
  }

  /**
   * Runs the drive at the desired velocity.
   *
   * @param speeds Speeds in meters/sec
   */
  public void runVelocity(ChassisSpeeds speeds) {
    SwerveModuleState[] setpointStates;
    double[] setpointAngularVelocities = null;

    // if steering adjustment is provided we override commanded omega
    // with the one compute based on the steering controller PID output
    if (steeringTurnAngleRadians.isPresent()) {
      // if we have teleop input on rotation and do not see the tag,
      // then do not override anything
      final var skipOverride =
          DriverStation.isTeleopEnabled()
              && !Misc.epsilonEquals(speeds.omegaRadiansPerSecond, 0)
              && !RobotState.getInstance().isTrackingTags();

      if (!skipOverride) {
        speeds.omegaRadiansPerSecond =
            steeringController.calculate(steeringTurnAngleRadians.getAsDouble(), 0)
                * getMaxAngularSpeedRadPerSec()
                * 0.75; // reduce maximum rotation speed
      }
    }

    // if all commanded velocities are effectively zero and
    // robot must be stopped in X wheel arrangement, use custom setpoints
    if (stopX
        && Math.abs(speeds.vxMetersPerSecond) <= Constants.epsilon
        && Math.abs(speeds.vyMetersPerSecond) <= Constants.epsilon
        && Math.abs(speeds.omegaRadiansPerSecond) <= Constants.epsilon) {
      setpointStates = stopXstates;
    } else {
      // calculate module setpoints
      final var discreteSpeeds = ChassisSpeeds.discretize(speeds, Constants.cyclePeriodSeconds);
      setpointStates = DriveConstants.kinematics.toSwerveModuleStates(discreteSpeeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(
          setpointStates, DriveConstants.drivetrainConfig.maxLinearSpeed());

      if (DriveConstants.useSecondOrderKinematics) {
        setpointAngularVelocities = calculateAngularVelocity(discreteSpeeds, setpointStates);
      }
    }

    // send setpoints to modules
    for (var i = 0; i < modules.length; ++i) {
      modules[i].runSetpoint(
          setpointStates[i],
          setpointAngularVelocities != null ? setpointAngularVelocities[i] : 0.0);

      updateModuleDelta(i, setpointStates[i]);
    }

    // log chassis speeds and setpoint states
    Logger.recordOutput("Drive/Speeds", speeds);
    Logger.recordOutput("Drive/SwerveStates/Setpoints", setpointStates);
    Logger.recordOutput("Drive/SwerveStates/Deltas", deltaStates);

    if (DriveConstants.useSecondOrderKinematics) {
      if (setpointAngularVelocities != null) {
        Logger.recordOutput("Drive/SwerveStates/Omegas", setpointAngularVelocities);
      }
    }
  }

  /** Stops the drive. */
  public void stop() {
    runVelocity(ZeroChassisSpeeds);
  }

  /**
   * Stops the drive and turns the modules to an X arrangement to resist movement. The modules will
   * return to their normal orientations the next time a nonzero velocity is requested.
   */
  public void stopWithX() {
    stopX = true;
    stop();
  }

  /**
   * If the robot is last stopped in an X arrangement, determines whether to resume normal movement
   * based on the magnitudes of input velocities.
   *
   * @param linearMagnitude linear velocity magnitude
   * @param angularMagnitude angular velocity magnitude
   */
  public void resumeFromX(double linearMagnitude, double angularMagnitude) {
    if (stopX) {
      final var movementRequested =
          Math.abs(linearMagnitude) > Constants.epsilon
              || Math.abs(angularMagnitude) > Constants.epsilon;

      if (movementRequested) {
        stopX = false;
      }
    }
  }

  /** Runs characterization at the commanded voltage. */
  public void runCharacterization(double volts) {
    for (var module : modules) {
      module.runCharacterization(volts, 0);
    }
  }

  /** Returns the average drive velocity in meters/sec. */
  public double getCharacterizationVelocity() {
    var driveVelocityAverage = 0.0;
    for (var module : modules) {
      driveVelocityAverage += module.getVelocityMetersPerSec();
    }
    return driveVelocityAverage / 4.0;
  }

  /** Returns the module states (turn angles and drive velocities) for all of the modules */
  @AutoLogOutput(key = "Drive/SwerveStates/Measured")
  private SwerveModuleState[] getModuleStates() {
    return measuredStates;
  }

  /** Returns the current odometry rotation */
  public Rotation2d getRotation() {
    return robotState.getEstimatedPose().getRotation();
  }

  /** Returns the yaw (rotational) velocity in radians per sec */
  public double getYawVelocityRadPerSec() {
    return gyroInputs.yawVelocityRadPerSec;
  }

  /**
   * Returns the measured X, Y, and theta field velocities in meters per sec. The components of the
   * twist are velocities and NOT changes in position.
   */
  public Twist2d getFieldVelocity() {
    return fieldVelocity;
  }

  /** Returns the maximum linear speed in meters per sec */
  public double getMaxLinearSpeedMetersPerSec() {
    return DriveConstants.drivetrainConfig.maxLinearSpeed();
  }

  /** Returns the maximum angular speed in radians per sec */
  public double getMaxAngularSpeedRadPerSec() {
    return DriveConstants.drivetrainConfig.maxAngularSpeed();
  }

  /**
   * Continously forces the drive subsystem to active maintain steering lock towards the goal by
   * giving it a turn angle to adjust the current rotation by.
   *
   * <p>This method can be used both with teleop and autonomous modes.
   */
  public void steeringLock(double turnAngleRadians) {
    steeringTurnAngleRadians = OptionalDouble.of(turnAngleRadians);
  }

  /** Releases steering lock. */
  public void steeringUnlock() {
    steeringTurnAngleRadians = OptionalDouble.empty();
  }

  /** Enables orientation lock. */
  public void enableOrientationLock(double angleRadians) {
    orientationLockEnabled = true;
    orientationLockAngleRad = angleRadians;
    orientationController.setSetpoint(angleRadians);
  }

  /** Toggles orientation lock. */
  public void toggleOrientationLock(double angleRadians) {
    if (orientationLockEnabled && orientationLockAngleRad == angleRadians) {
      disableOrientationLock();
    } else {
      enableOrientationLock(angleRadians);
    }
  }

  /** Disables orientation lock. */
  public void disableOrientationLock() {
    orientationLockEnabled = false;
    orientationLockAngleRad = 0;
    orientationController.setSetpoint(0);
  }

  /** Returns rotation input required to maintain orientation lock. */
  public double getOrientationLockRotation() {
    return orientationController.calculate(getRotation().getRadians());
  }

  /** Returns an indicator that orientation lock is enabled. */
  @AutoLogOutput(key = "Drive/OrientationLock/Enabled")
  public boolean hasOrientationLock() {
    return orientationLockEnabled;
  }

  /** Gets robot pose. */
  public Pose2d getPose() {
    return robotState.getEstimatedPose();
  }

  /** Sets robot pose. */
  public void setPose(Pose2d initialPose) {
    robotState.resetPose(initialPose);
  }

  /** Reset encoders. */
  public void reset() {
    for (var module : modules) {
      module.resetEncoders();
    }
  }

  /**
   * Provides an "escape hatch" command to manually orient all modules to the specified rotation.
   */
  public Command orientModules(Rotation2d orientation) {
    return orientModules(new Rotation2d[] {orientation, orientation, orientation, orientation});
  }

  /** Provides an "escape hatch" command to manually orient modules to the specified rotations. */
  public Command orientModules(Rotation2d[] orientations) {
    final var tolerance = Math.toRadians(3.0);
    return run(() -> {
          for (int i = 0; i < modules.length; ++i) {
            modules[i].runSetpoint(new SwerveModuleState(0.0, orientations[i]), 0);
          }
        })
        .until(
            () -> {
              for (var i = 0; i < modules.length; ++i) {
                final var delta =
                    Misc.clampAngle(
                        modules[i].getAngle().getRadians() - orientations[i].getRadians());

                if (!Misc.epsilonEquals(delta, 0, tolerance)
                    && !Misc.epsilonEquals(delta, Math.PI, tolerance)) {
                  return false;
                }
              }

              return true;
            })
        .withName("Orient Modules");
  }
}
