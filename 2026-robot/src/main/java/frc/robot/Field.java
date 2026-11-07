package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

/** Field-related functionality. Blue alliance origin. */
public final class Field {
  /** Robot bumper size (assumes square robot) */
  public static final double kBumperSize = Units.inchesToMeters(34);

  /** Field length (X) */
  public static final double kFieldLength = Units.inchesToMeters(651.2);

  /** Field width (Y) */
  public static final double kFieldWidth = Units.inchesToMeters(317.7);

  /** Field half length (X) */
  public static final double kFieldHalfLength = kFieldLength / 2.0;

  /** Field half width (Y) */
  public static final double kFieldHalfWidth = kFieldWidth / 2.0;

  /** Middle of the starting line */
  public static final double kStartingLineX = 4;

  /** Neutral zone near side */
  public static final double kNeutralZoneNear = kFieldHalfLength - Units.inchesToMeters(120);

  /** Neutral zone far side */
  public static final double kNeutralZoneFar = kFieldHalfLength + Units.inchesToMeters(120);

  /** Middle of the hub center line */
  public static final double kHubLineX = 4.625;

  /** Effective zone for hub shooting, requiring bumpers to be partially within the alliance zone */
  public static final double kShootingZoneX =
      kStartingLineX + kBumperSize / 2 - Units.inchesToMeters(4);

  /** Center of the hub */
  public static final Translation2d kHubCenter = new Translation2d(kHubLineX, kFieldHalfWidth);

  /** Center of the tower, face through the middle of the climb */
  public static final Translation2d kTowerCenter =
      new Translation2d(Units.inchesToMeters(40 + 3.51 * 0.5), Units.inchesToMeters(147.47));

  /** Tower width including uprights */
  public static final double kTowerWidth = Units.inchesToMeters(32.250 + 3.0);

  /** Tower safe radius: 6" to clear protruding rungs */
  public static final double kTowerSafeRadius =
      (kTowerWidth + kBumperSize) * 0.5 + Units.inchesToMeters(6);

  /** Trench width (approximate) in meters */
  public static final double kTrenchWidth = Units.inchesToMeters(50);

  /** Center of the right (lower) trench (Blue) */
  public static final Translation2d kBlueTrenchRightCenter =
      new Translation2d(kHubLineX, kTrenchWidth / 2);

  /** Center of the left (upper) trench (Blue) */
  public static final Translation2d kBlueTrenchLeftCenter =
      new Translation2d(kHubLineX, kFieldWidth - kTrenchWidth / 2);

  /** Center of the right (lower) trench (Red) */
  public static final Translation2d kRedTrenchRightCenter =
      new Translation2d(kFieldLength - kHubLineX, kFieldWidth - kTrenchWidth / 2);

  /** Center of the left (upper) trench (Red) */
  public static final Translation2d kRedTrenchLeftCenter =
      new Translation2d(kFieldLength - kHubLineX, kTrenchWidth / 2);

  /** Approximate passing point as measured from the corresponding corner of the field. */
  public static final Translation2d kDefaultPassingPoint = new Translation2d(2, 2.5);

  private Field() {
    throw new AssertionError();
  }

  public static enum FieldType {
    MIRRORED {
      public double flipX(double x) {
        return kFieldLength - x;
      }

      public double flipY(double y) {
        return y;
      }

      public double flipHeading(double heading) {
        return Math.PI - heading;
      }
    },
    ROTATED {
      public double flipX(double x) {
        return kFieldLength - x;
      }

      public double flipY(double y) {
        return kFieldWidth - y;
      }

      public double flipHeading(double heading) {
        return Math.PI + heading;
      }
    };

    public abstract double flipX(double x);

    public abstract double flipY(double y);

    public abstract double flipHeading(double heading);
  }

  public static final FieldType kType = FieldType.ROTATED;

  /** Determines whether field coordinates should be flipped, aka being on red alliance. */
  public static boolean shouldFlip() {
    return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red;
  }

  public static double flipX(double x) {
    return kType.flipX(x);
  }

  public static double flipY(double y) {
    return kType.flipY(y);
  }

  public static double flipHeading(double heading) {
    return kType.flipHeading(heading);
  }

  public static Translation2d flip(Translation2d translation) {
    return new Translation2d(flipX(translation.getX()), flipY(translation.getY()));
  }

  public static Rotation2d flip(Rotation2d rotation) {
    return switch (kType) {
      case MIRRORED -> new Rotation2d(-rotation.getCos(), rotation.getSin());
      case ROTATED -> new Rotation2d(-rotation.getCos(), -rotation.getSin());
    };
  }

  public static Pose2d flip(Pose2d pose) {
    return new Pose2d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  public static Translation3d flip(Translation3d translation) {
    return new Translation3d(
        flipX(translation.getX()), flipY(translation.getY()), translation.getZ());
  }

  public static Rotation3d flip(Rotation3d rotation) {
    return new Rotation3d(
        rotation.getX(), rotation.getY(), flip(rotation.toRotation2d()).getRadians());
  }

  public static Pose3d flip(Pose3d pose) {
    return new Pose3d(flip(pose.getTranslation()), flip(pose.getRotation()));
  }

  /** Transforms the "blue-origin" pose based on the current alliance. */
  public static Pose2d transform(Pose2d v) {
    return shouldFlip() ? flip(v) : v;
  }

  /** Transforms the "blue-origin" coordinates based on the current alliance. */
  public static Translation2d transform(Translation2d v) {
    return shouldFlip() ? flip(v) : v;
  }

  /** Transforms the "blue-origin" X coordinate based on the current alliance. */
  public static double transformX(double v) {
    return shouldFlip() ? flipX(v) : v;
  }

  /** Transforms the "blue-origin" Y coordinate based on the current alliance. */
  public static double transformY(double v) {
    return shouldFlip() ? flipY(v) : v;
  }

  private static double kTrenchZoneThreshold = Units.inchesToMeters(12);

  /**
   * Determines whether the specified translation (XY) falls within either trench zone on either
   * alliance side close to the center line +- threshold.
   */
  public static boolean isInTrenchZone(Translation2d xy) {
    final var x = xy.getX();
    final var y = xy.getY();

    if (y < kTrenchWidth || y > kFieldWidth - kTrenchWidth) {
      // blue alliance side
      if (Math.abs(kHubLineX - x) < kTrenchZoneThreshold) {
        return true;
      }

      // red alliance side
      if (Math.abs(kFieldLength - kHubLineX - x) < kTrenchZoneThreshold) {
        return true;
      }
    }

    return false;
  }

  /**
   * Extends {@link #isInTrenchZone(Translation2d)} with predictive behavior based on current
   * velocity and time horizon.
   */
  public static boolean isInTrenchZone(Translation2d xy, ChassisSpeeds fieldSpeeds, double dt) {
    return isInTrenchZone(xy)
        || isInTrenchZone(
            new Translation2d(
                xy.getX() + fieldSpeeds.vxMetersPerSecond * dt,
                xy.getY() + fieldSpeeds.vyMetersPerSecond * dt));
  }

  /**
   * Determines whether the specified robot pose's translation falls within the shooting zone aka
   * bumpers partially within the alliance zone
   */
  public static boolean isInShootingZone(Translation2d xy) {
    final var x = xy.getX();
    return shouldFlip() ? x > kFieldLength - kShootingZoneX : x < kShootingZoneX;
  }

  /**
   * Determines whether the specified robot pose's translation falls within the passing zone,
   * neutral zone onwards.
   */
  public static boolean isInPassingZone(Translation2d xy) {
    final var x = xy.getX();
    return shouldFlip() ? x <= kFieldLength - kNeutralZoneNear : x >= kNeutralZoneNear;
  }

  /**
   * Determines whether the specified robot pose is safely away from the tower to lower the climb.
   */
  public static boolean isAwayFromTower(Pose2d pose) {
    final var xy = pose.getTranslation();

    if (shouldFlip()) {
      if (xy.getX() < kFieldLength - kTowerCenter.getX() - kBumperSize * 0.5) {
        return true;
      }
    } else {
      if (xy.getX() > kTowerCenter.getX() + kBumperSize * 0.5) {
        return true;
      }
    }

    final var dy = Math.abs(transformY(kTowerCenter.getY()) - pose.getY());
    return dy > kTowerSafeRadius;
  }

  public static enum FieldZone {
    kAlliance,
    kNeutral,
    kOpponentAlliance,
    kTrenchLeft,
    kTrenchRight,
    kBumpLeft,
    kBumpRight,
  }

  /** Returns the zone the pose translation falls within. */
  public static FieldZone getZone(Translation2d xy) {
    // transform coordinates to be expressed in the blue-alliance origin
    final var x = transformX(xy.getX());
    final var y = transformY(xy.getY());

    // use X coordinate to dissect the field first
    if (x <= kStartingLineX) {
      return FieldZone.kAlliance;
    }

    if (x >= kFieldLength - kStartingLineX) {
      return FieldZone.kOpponentAlliance;
    }

    if (x >= kNeutralZoneNear && x <= kNeutralZoneFar) {
      return FieldZone.kNeutral;
    }

    // we are within trench/bump, use Y coordinate to dissect
    if (y <= kTrenchWidth) {
      return FieldZone.kTrenchRight;
    }

    if (y >= kFieldWidth - kTrenchWidth) {
      return FieldZone.kTrenchLeft;
    }

    return y <= kFieldHalfWidth ? FieldZone.kBumpRight : FieldZone.kBumpLeft;
  }
}
