package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Supplier;

public final class FieldConstants {
  public static final double fieldLengthMeters = 16.5410515;
  public static final double fieldCenterMeters = fieldLengthMeters / 2;

  /** Returns the pose mirrored across the field midline. */
  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(
        fieldLengthMeters - pose.getX(),
        pose.getY(),
        Rotation2d.fromRadians(Math.PI - pose.getRotation().getRadians()));
  }

  /** Returns the pose corrected based on the selected alliance. */
  public static Supplier<Pose2d> getFieldPose(Pose2d pose) {
    return () -> {
      final var flipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;
      return flipped ? flip(pose) : pose;
    };
  }

  // in meters (blue alliance wall)
  // spotless:off
  public static final Pose2d initialAmp = new Pose2d(0.785, 6.711, Rotation2d.fromDegrees(60));
  public static final Pose2d initialCenter = new Pose2d(1.383, 5.576, Rotation2d.fromDegrees(0));
  public static final Pose2d initialLoad = new Pose2d(0.809, 4.416, Rotation2d.fromDegrees(-60));

  public static final Pose2d podiumShot = new Pose2d(2.3, 4.4, Rotation2d.fromDegrees(-27));

  public static final Pose2d note1 = new Pose2d(2.89, 7.019, Rotation2d.fromDegrees(30));
  public static final Pose2d note2 = new Pose2d(2.89, 5.56, Rotation2d.fromDegrees(0));
  public static final Pose2d note3 = new Pose2d(2.89, 4.11, Rotation2d.fromDegrees(-30));

  public static final Pose2d centralNote1 = new Pose2d(fieldCenterMeters, 7.47, Rotation2d.fromDegrees(15));
  public static final Pose2d centralNote2 = new Pose2d(fieldCenterMeters, 5.79, Rotation2d.fromDegrees(-15));
  public static final Pose2d centralNote3 = new Pose2d(fieldCenterMeters, 4.11, Rotation2d.fromDegrees(-15));
  public static final Pose2d centralNote4 = new Pose2d(fieldCenterMeters, 2.43, Rotation2d.fromDegrees(15));
  public static final Pose2d centralNote5 = new Pose2d(fieldCenterMeters, 0.75, Rotation2d.fromDegrees(-15));
  // spotless:on
}
