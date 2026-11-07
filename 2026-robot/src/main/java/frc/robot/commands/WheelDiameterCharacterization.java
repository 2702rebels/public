package frc.robot.commands;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.lib.Tunable;
import frc.robot.subsystems.CommandSwerveDrivetrain;

/**
 * Implements characterization routine for wheel radius measurement.
 *
 * <p>The idea and skeleton implementation is borrowed from the FRC6328 (Mechanical Advantage) team.
 */
public class WheelDiameterCharacterization extends Command {
  private static final Tunable.DoubleValue omega =
      Tunable.value("WheelDiameterCharacterization/Omega", 1.0);

  private static final double turnRadius = TunerConstants.turnRadius.in(Meters);
  private static final double wheelRadius = TunerConstants.wheelRadius.in(Meters);

  // spotless:off
  private final NetworkTable table = NetworkTableInstance.getDefault().getTable("Drive").getSubTable("WheelCharacterization");
  private final DoublePublisher wheelRadiansTopic = table.getDoubleTopic("Wheel Radians").publish();
  private final DoublePublisher rotationRadiansTopic = table.getDoubleTopic("Rotation Radians").publish();
  private final DoublePublisher radiusInchesTopic = table.getDoubleTopic("Estimated Radius").publish();
  // spotless:on

  private final CommandSwerveDrivetrain drivetrain;
  private final SwerveRequest.RobotCentric driveRequest = new SwerveRequest.RobotCentric();

  private final SlewRateLimiter omegaLimiter = new SlewRateLimiter(1.0);

  private final double[] initialWheelDistanceMeters = new double[4];
  private final int direction;
  private double lastRotationRadians = 0.0;
  private double totalRotationRadians = 0.0;
  private double estimatedWheelRadius = 0.0;

  public WheelDiameterCharacterization(CommandSwerveDrivetrain drivetrain, int direction) {
    this.drivetrain = drivetrain;
    this.direction = direction;
    addRequirements(drivetrain);
  }

  @Override
  public void initialize() {
    System.out.println("Wheel characterization initialized");
    final var state = drivetrain.getState();

    totalRotationRadians = 0.0;
    lastRotationRadians = state.RawHeading.getRadians();

    // record initial distance traveled by each wheel
    for (var i = 0; i < state.ModulePositions.length; ++i) {
      initialWheelDistanceMeters[i] = state.ModulePositions[i].distanceMeters;
    }

    omegaLimiter.reset(0);
  }

  @Override
  public void execute() {
    final var omegaRadiansPerSecond = omegaLimiter.calculate(direction * omega.get());
    drivetrain.setControl(driveRequest.withRotationalRate(omegaRadiansPerSecond));

    final var state = drivetrain.getState();
    final var rotationRadians = state.RawHeading.getRadians();

    // determine cumulative rotation angle by the robot
    totalRotationRadians += MathUtil.angleModulus(rotationRadians - lastRotationRadians);
    lastRotationRadians = rotationRadians;

    // determine distance traveled by wheels (averaging across all wheels)
    double averageWheelDistanceMeters = 0.0;
    for (var i = 0; i < state.ModulePositions.length; ++i) {
      averageWheelDistanceMeters +=
          Math.abs(state.ModulePositions[i].distanceMeters - initialWheelDistanceMeters[i]);
    }
    averageWheelDistanceMeters /= initialWheelDistanceMeters.length;

    // estimate effective wheel radius based on the drive radius (from the chassis geometry),
    // using the current wheel radius to convert distance traveled back to wheel rotations
    // measured in radians
    final var averageWheelDistanceRadians = averageWheelDistanceMeters / wheelRadius;
    estimatedWheelRadius = (totalRotationRadians * turnRadius) / averageWheelDistanceRadians;

    wheelRadiansTopic.set(Math.abs(averageWheelDistanceRadians));
    rotationRadiansTopic.set(Math.abs(totalRotationRadians));
    radiusInchesTopic.set(Units.metersToInches(Math.abs(estimatedWheelRadius)));
  }

  @Override
  public void end(boolean interrupted) {
    if (Math.abs(totalRotationRadians) <= Math.PI * 2.0) {
      System.out.println("Insufficient data collected for characterization");
    } else {
      System.out.println(
          "Effective wheel radius: "
              + Units.metersToInches(Math.abs(estimatedWheelRadius))
              + " inches");
    }
  }
}
