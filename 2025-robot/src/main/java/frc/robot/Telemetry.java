package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import frc.robot.generated.TunerConstants;

public class Telemetry {
  private final NetworkTableInstance nt = NetworkTableInstance.getDefault();
  private final NetworkTable driveStateTable = nt.getTable("DriveState");

  private final StructPublisher<Pose2d> drivePose =
      driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();

  private final StructPublisher<ChassisSpeeds> driveSpeeds =
      driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();

  private final StructArrayPublisher<SwerveModuleState> driveModuleStates =
      driveStateTable.getStructArrayTopic("ModuleStates", SwerveModuleState.struct).publish();

  private final StructArrayPublisher<SwerveModuleState> driveModuleTargets =
      driveStateTable.getStructArrayTopic("ModuleTargets", SwerveModuleState.struct).publish();

  private final DoubleArrayPublisher driveModuleSlipRatios =
      driveStateTable.getDoubleArrayTopic("ModuleSlipRatios").publish();

  private final DoublePublisher driveTimestamp =
      driveStateTable.getDoubleTopic("Timestamp").publish();

  private final DoublePublisher driveOdometryFrequency =
      driveStateTable.getDoubleTopic("OdometryFrequency").publish();

  private final double[] poseArray = new double[3];
  private final double[] moduleStatesArray = new double[8];
  private final double[] moduleTargetsArray = new double[8];
  private final double[] moduleSlipRatiosArray = new double[4];

  private static final double kStationaryTolerance = 0.3; // speed (m/s)

  public Telemetry(boolean loggingEnabled) {
    if (loggingEnabled) {
      SignalLogger.setPath("/u/logs/ctre/");
      SignalLogger.start();
    }
  }

  /** Accepts the swerve drive state and log via SignalLogger and NT. */
  public void update(SwerveDriveState state) {
    drivePose.set(state.Pose);
    driveSpeeds.set(state.Speeds);
    driveModuleStates.set(state.ModuleStates);
    driveModuleTargets.set(state.ModuleTargets);
    driveTimestamp.set(state.Timestamp);
    driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

    computeSlipRatios(state, moduleSlipRatiosArray);
    driveModuleSlipRatios.set(moduleSlipRatiosArray);

    // signal logger
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

  /** Computes slip ratios for each module and populates the array. */
  public static void computeSlipRatios(SwerveDriveState state, double[] slipRatios) {
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
      if (MathUtil.isNear(0, robotVelocityNorm, kStationaryTolerance)) {
        slipRatios[i] =
            MathUtil.isNear(0, moduleTranslationVelocity.getNorm(), kStationaryTolerance) ? 0 : 1;
      } else {
        // technically we should compute projections on the robot IMU velocity vector
        // but all these vectors should be colinear unless the robot is falling apart
        slipRatios[i] =
            (moduleTranslationVelocity.getNorm() - robotVelocityNorm) / robotVelocityNorm;
      }
    }
  }
}
