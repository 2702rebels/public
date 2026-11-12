package frc.robot;

import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

public final class GameConstants {
  /** Assumes robot bumpers are square. Outer width/height of the robot with bumpers on. */
  public static final double kBumperDimensions = Units.inchesToMeters(34);

  /** X coordinate (meters) of the robot center when at climb ready position. */
  public static final double kClimbPositionX = 8.285;

  /** X coordinate (meters) of the robot center when at net scoring ready position. */
  public static final double kNetScoringPositionX = 8.3;

  /**
   * Robot poses for scoring at each reef branch starting at the face closest to the fall in
   * counterclockwise order. Index corresponds to the A, B, ..., K, L references in the game manual.
   */
  public static final Pose2d[] reefScoringPosesL2 = new Pose2d[12];

  /** Robot poses for scoring at L3 level (different gap). */
  public static final Pose2d[] reefScoringPosesL3 = new Pose2d[12];

  /** Robot poses for scoring at L4 level (different gap). */
  public static final Pose2d[] reefScoringPosesL4 = new Pose2d[12];

  private static final double kBranchShift = Units.inchesToMeters(13) / 2;

  private static Translation2d getScoringOffset(Rotation2d rotation, int index, double gap) {
    return new Translation2d(kBumperDimensions / 2 + gap, kBranchShift * (index % 2 == 0 ? -1 : 1))
        .rotateBy(rotation);
  }

  static {
    final var rotation180 = Rotation2d.fromDegrees(180);
    final var gapL2 = Units.inchesToMeters(10);
    final var gapL3 = Units.inchesToMeters(8);
    final var gapL4 = Units.inchesToMeters(1);

    for (int i = 0; i < 12; ++i) {
      final var face = FieldConstants.Reef.faces[i / 2];
      final var faceRotation = face.getRotation();

      final var offsetL2 = getScoringOffset(faceRotation, i, gapL2);
      final var offsetL3 = getScoringOffset(faceRotation, i, gapL3);
      final var offsetL4 = getScoringOffset(faceRotation, i, gapL4);

      final var rotation = faceRotation.rotateBy(rotation180);

      reefScoringPosesL2[i] = new Pose2d(face.getTranslation().plus(offsetL2), rotation);
      reefScoringPosesL3[i] = new Pose2d(face.getTranslation().plus(offsetL3), rotation);
      reefScoringPosesL4[i] = new Pose2d(face.getTranslation().plus(offsetL4), rotation);

      // System.out.println(
      //     i
      //         + ": "
      //         + String.format("x=%.3f", reefScoringPoses[i].getX())
      //         + String.format(" y=%.3f", reefScoringPoses[i].getY())
      //         + String.format(" r=%.0f", reefScoringPoses[i].getRotation().getDegrees()));
    }
  }

  /** Safety margin to use for safe field perimeter. */
  public static final double kFieldPerimeterSafetyMargin =
      kBumperDimensions / 2 + Units.inchesToMeters(12);

  /** Determines if the pose is at the desired climb position. */
  public static boolean isAtClimbPosition(Pose2d pose) {
    final var x = FieldConstants.fieldPose.apply(pose).getX();
    return MathUtil.isNear(kClimbPositionX, x, Units.inchesToMeters(1));
  }

  /** Determines if the pose is at the desired net scoring position. */
  public static boolean isAtNetScoringPosition(Pose2d pose) {
    final var x = FieldConstants.fieldPose.apply(pose).getX();
    return MathUtil.isNear(kNetScoringPositionX, x, Units.inchesToMeters(1));
  }

  /** Determines if the pose is within safe field perimeter to be used for auto. */
  public static boolean withinAutoFieldPerimeter(Pose2d pose) {
    // here logic is defined for blue origin side, since we don't want to cross the center line,
    // flip the supplied pose if the robot is on the red alliance side
    final var adjustedPose = FieldConstants.fieldPose.apply(pose);

    final var x = adjustedPose.getX();
    final var y = adjustedPose.getY();
    final var maxX = FieldConstants.kStartingLineX.in(Meters);
    final var maxY = FieldConstants.kFieldWidth.in(Meters);

    if (x < kFieldPerimeterSafetyMargin
        || x > maxX - kFieldPerimeterSafetyMargin
        || y < kFieldPerimeterSafetyMargin
        || y > maxY - kFieldPerimeterSafetyMargin) {
      return false;
    }

    // account for loading station corners
    // Bottom: (0, 1.2m), (1.7m, 0)
    // LineEq: 0.7x + y - 1.2 < 0
    if (0.7 * x + y - 1.2 < kFieldPerimeterSafetyMargin
        || -0.7 * x + y + 1.2 - maxY > -kFieldPerimeterSafetyMargin) {
      return false;
    }

    return true;
  }

  /** Field quadrant. Left or right based on the alliance perspective. */
  public enum FieldQuadrant {
    BlueLeft,
    BlueRight,
    RedLeft,
    RedRight,
  }

  /** Returns the field quadrant of the pose. */
  public static FieldQuadrant getQuadrant(Pose2d pose) {
    final var y = pose.getY();
    if (pose.getX() < FieldConstants.kFieldHalfLength.in(Meters)) {
      return y < FieldConstants.kFieldHalfWidth.in(Meters)
          ? FieldQuadrant.BlueRight
          : FieldQuadrant.BlueLeft;
    } else {
      return y < FieldConstants.kFieldHalfWidth.in(Meters)
          ? FieldQuadrant.RedLeft
          : FieldQuadrant.RedRight;
    }
  }
}
