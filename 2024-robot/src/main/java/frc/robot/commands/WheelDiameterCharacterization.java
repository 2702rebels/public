package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.DriveConstants;
import frc.robot.util.LoggedTunableNumber;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

/**
 * Implements characterization routine for wheel radius measurement.
 *
 * <p>The idea and skeleton implementation is borrowed from the FRC6328 (Mechanical Advantage) team.
 */
public class WheelDiameterCharacterization extends Command {
  private static final LoggedTunableNumber omega =
      new LoggedTunableNumber("Drive/Wheel Diameter/Omega", 0.3);

  private static final double driveRadius = DriveConstants.drivetrainConfig.driveBaseRadius();
  private static final DoubleSupplier rotationRadiansSupplier =
      () -> RobotState.getInstance().getOdometryPose().getRotation().getRadians();

  private final Drive drive;
  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private final double[] initialWheelDistanceMeters = new double[4];
  private final int direction;
  private double lastRotationRadians = 0.0;
  private double totalRotationRadians = 0.0;
  private double estimatedWheelRadius = 0.0;

  public WheelDiameterCharacterization(Drive drive, int direction) {
    this.drive = drive;
    this.direction = direction;
    addRequirements(drive);
  }

  @Override
  public void initialize() {
    lastRotationRadians = rotationRadiansSupplier.getAsDouble();
    totalRotationRadians = 0.0;

    // remember initial distance travelled by each wheel
    final var positions = RobotState.getInstance().getWheelPositions();
    for (var i = 0; i < initialWheelDistanceMeters.length; ++i) {
      initialWheelDistanceMeters[i] = positions.positions[i].distanceMeters;
    }

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    final var omegaRadiansPerSecond = omegaLimiter.calculate(direction * omega.get());
    drive.runVelocity(new ChassisSpeeds(0, 0, omegaRadiansPerSecond));

    // determine cumulative rotation angle by the robot
    totalRotationRadians +=
        MathUtil.angleModulus(rotationRadiansSupplier.getAsDouble() - lastRotationRadians);
    lastRotationRadians = rotationRadiansSupplier.getAsDouble();

    // determine distance travelled by wheels (averaging across all wheels)
    double averageWheelDistanceMeters = 0.0;
    final var positions = RobotState.getInstance().getWheelPositions();
    for (var i = 0; i < initialWheelDistanceMeters.length; ++i) {
      averageWheelDistanceMeters +=
          Math.abs(positions.positions[i].distanceMeters - initialWheelDistanceMeters[i]);
    }
    averageWheelDistanceMeters /= initialWheelDistanceMeters.length;

    // estimate effective wheel radius based on the drive radius (from the chassis geometry),
    // using the current wheel radius to convert distance travelled back to wheel rotations
    // measured in radians
    final var averageWheelDistanceRadians =
        2 * averageWheelDistanceMeters / DriveConstants.drivetrainConfig.wheelDiameter();

    estimatedWheelRadius = (totalRotationRadians * driveRadius) / averageWheelDistanceRadians;

    // spotless:off
    Logger.recordOutput("Drive/Wheel Diameter/Wheel Radians", averageWheelDistanceRadians);
    Logger.recordOutput("Drive/Wheel Diameter/Rotation Radians", totalRotationRadians);
    Logger.recordOutput("Drive/Wheel Diameter/Estimated Diameter", Units.metersToInches(estimatedWheelRadius * 2));
    // spotless:on
  }

  @Override
  public void end(boolean interrupted) {
    if (totalRotationRadians <= Math.PI * 2.0) {
      System.out.println("Insufficient data collected for characterization");
    } else {
      System.out.println(
          "Effective wheel diameter: "
              + Units.metersToInches(estimatedWheelRadius * 2)
              + " inches");
    }
  }
}
