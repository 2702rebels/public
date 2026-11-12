package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.AngleSlewRateLimiter;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Misc;
import java.util.function.DoubleSupplier;

public class DriveTeleop extends Command {
  private final Drive drive;
  private final DoubleSupplier xSupplier;
  private final DoubleSupplier ySupplier;
  private final DoubleSupplier rotationSupplier;

  private static final LoggedTunableNumber deadband =
      new LoggedTunableNumber("Drive/Teleop/Deadband", 0.2);

  private static final LoggedTunableNumber slewDirection =
      new LoggedTunableNumber("Drive/Teleop/Slew/Direction", Math.PI / 2); // radians per second

  private static final LoggedTunableNumber slewAccelerate =
      new LoggedTunableNumber("Drive/Teleop/Slew/Accelerate", 0.8); // percent per second (1 = 100%)

  private static final LoggedTunableNumber slewDecelerate =
      new LoggedTunableNumber("Drive/Teleop/Slew/Decelerate", 2.0); // percent per second (1 = 100%)

  private static final LoggedTunableNumber slewRotation =
      new LoggedTunableNumber("Drive/Teleop/Slew/Rotation", 1.5); // percent per second (1 = 100%)

  private double actualLinearMagnitude;
  private double actualHeadingRad;
  private double inputHeadingRad;
  private AngleSlewRateLimiter directionRateLimiter;
  private SlewRateLimiter magnitudeRateLimiter;
  private SlewRateLimiter rotationRateLimiter;

  public DriveTeleop(
      Drive drive,
      DoubleSupplier xSupplier,
      DoubleSupplier ySupplier,
      DoubleSupplier rotationSupplier) {
    addRequirements(drive);
    this.drive = drive;
    this.xSupplier = xSupplier;
    this.ySupplier = ySupplier;
    this.rotationSupplier = rotationSupplier;

    directionRateLimiter = new AngleSlewRateLimiter(slewDirection.get());
    rotationRateLimiter = new SlewRateLimiter(slewRotation.get());
    magnitudeRateLimiter =
        new SlewRateLimiter(slewAccelerate.get(), -Math.abs(slewDecelerate.get()), 0);
  }

  @Override
  public void execute() {
    final var x = xSupplier.getAsDouble();
    final var y = ySupplier.getAsDouble();
    var rotation = MathUtil.applyDeadband(rotationSupplier.getAsDouble(), deadband.get());

    // disable orientation if commanded to rotate
    if (Math.abs(rotation) > Constants.epsilon && drive.hasOrientationLock()) {
      drive.disableOrientationLock();
    }

    // apply deadband
    final var linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), deadband.get());

    // maintain last heading at stop / very low speeds to prevent
    // abrupt changes to lateral direction that causes the robot
    // to jerk when the joystick is released
    if (Math.abs(linearMagnitude) > 0.01) {
      inputHeadingRad = Math.atan2(x, y);
    }

    // reset X wheel formation when input is received
    drive.resumeFromX(linearMagnitude, rotation);

    // apply rate limiting to rotation
    rotation = rotationRateLimiter.calculate(rotation);

    // use PID for orientation
    if (drive.hasOrientationLock()) {
      rotation = drive.getOrientationLockRotation();
    }

    // make heading slew rate dependent on the magnitude of lateral "speed"
    // when the robot is moving slowly we should allow for much faster changes
    // to the directional heading, while clamping it down at higher speeds
    //
    // here we calculate a "damping" factor as the inverse of the lateral "speed";
    // at maximum "speed" (1) the factor is 1, forcing the limiter to apply
    // the maximum constraints; as the speed decreases the factor increases
    // effectively increasing the maximum allowed slew rate
    final var factor =
        Math.abs(actualLinearMagnitude) < Constants.epsilon
            ? 1000.0
            : Math.abs(1.0 / actualLinearMagnitude);

    // we consider three scenarios based on the angular sectors where
    // new heading falls in relative to the current heading (last commanded):
    // 1. the 80° sector in front
    // 2. the 80° sector in rear
    // 3. the 100° left and right sectors
    //
    // - in the first case we continue maintaining commanded speed and steer
    // as usual (subject to dumping factor);
    // - in the second case we keep current heading while slowing down to
    // a full stop then flip the heading 180° and start accelerating to
    // the commanded speed (places us back to case 1.)
    // - in the third case we start slowing down to a full stop while steering
    // towards new heading until we are back to the case 1.

    final var headingDelta = Math.abs(Misc.clampAngle(inputHeadingRad - actualHeadingRad));

    final var sectorF = 0.22 * Math.PI;
    final var sectorR = 0.78 * Math.PI;

    if (headingDelta < sectorF) {
      actualLinearMagnitude = magnitudeRateLimiter.calculate(linearMagnitude);
      actualHeadingRad = directionRateLimiter.calculate(inputHeadingRad, factor);
    } else if (headingDelta > sectorR) {
      if (Math.abs(actualLinearMagnitude) > Constants.epsilon) {
        // keep heading
        actualLinearMagnitude = magnitudeRateLimiter.calculate(0.0);
      } else {
        actualLinearMagnitude = magnitudeRateLimiter.calculate(linearMagnitude);
        actualHeadingRad = Misc.clampAngle(actualHeadingRad + Math.PI);
      }
    } else {
      actualLinearMagnitude = magnitudeRateLimiter.calculate(0.0);
      actualHeadingRad = directionRateLimiter.calculate(inputHeadingRad, factor);
    }

    // convert the commanded speeds into the correct units for the drivetrain
    final var maxLinearSpeed = drive.getMaxLinearSpeedMetersPerSec();
    final var vxMetersPerSecond =
        actualLinearMagnitude * Math.cos(actualHeadingRad) * maxLinearSpeed;
    final var vyMetersPerSecond =
        actualLinearMagnitude * Math.sin(actualHeadingRad) * maxLinearSpeed;

    final var omegaRadiansPerSecond = rotation * drive.getMaxAngularSpeedRadPerSec();

    final var speeds =
        ChassisSpeeds.fromFieldRelativeSpeeds(
            vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, drive.getRotation());

    drive.runVelocity(speeds);
  }

  @Override
  public void initialize() {
    actualLinearMagnitude = 0;
    actualHeadingRad = 0;
    inputHeadingRad = 0;
  }

  @Override
  public void end(boolean interrupted) {
    drive.stop();
  }
}
