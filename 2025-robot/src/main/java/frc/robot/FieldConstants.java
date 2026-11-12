package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.util.ChoreoAllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import java.util.function.Function;

/**
 * Field-related constants. Blue alliance origin.
 *
 * <p>All distance measurements are in meters unless specified otherwise.
 */
public final class FieldConstants {
  /** Field length (X) */
  public static final Distance kFieldLength = Inches.of(690.876);

  /** Field width (Y) */
  public static final Distance kFieldWidth = Inches.of(317);

  /** Center of the starting line */
  public static final Distance kStartingLineX = Inches.of(298.5);

  /** Field half length (X) */
  public static final Distance kFieldHalfLength = kFieldLength.div(2);

  /** Field half width (Y) */
  public static final Distance kFieldHalfWidth = kFieldWidth.div(2);

  public static final class Reef {
    public static final Translation2d center =
        new Translation2d(Units.inchesToMeters(176.75), kFieldWidth.in(Meters) / 2.0);

    /**
     * Center points of each face starting at the face closest to the wall in counterclockwise
     * order. Index corresponds to the AB, CD, EF, GH, IJ, KL references in the game manual.
     *
     * <p>These should coincide with the AprilTag locations on the reef.
     */
    public static final Pose2d[] faces = new Pose2d[6];

    static {
      // spotless:off
      faces[0] = new Pose2d(Units.inchesToMeters(144.00), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(180));
      faces[1] = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(240));
      faces[2] = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(130.17), Rotation2d.fromDegrees(300));
      faces[3] = new Pose2d(Units.inchesToMeters(209.49), Units.inchesToMeters(158.50), Rotation2d.fromDegrees(0));
      faces[4] = new Pose2d(Units.inchesToMeters(193.10), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(60));
      faces[5] = new Pose2d(Units.inchesToMeters(160.39), Units.inchesToMeters(186.83), Rotation2d.fromDegrees(120));
      // spotless:on
    }
  }

  public enum ReefHeight {
    L4(Units.inchesToMeters(72), -90),
    L3(Units.inchesToMeters(47.625), -35),
    L2(Units.inchesToMeters(31.875), -35),
    L1(Units.inchesToMeters(18), 0);

    ReefHeight(double height, double pitchDegrees) {
      this.height = height;
      this.pitchDegrees = pitchDegrees;
    }

    public final double height;
    public final double pitchDegrees;
  }

  public static final class Barge {
    /**
     * Center points of each cage starting at the one closes to the field center and going outward.
     */
    public static final Translation2d[] cages = new Translation2d[3];

    static {
      // spotless:off
      cages[0] = new Translation2d(kFieldLength.in(Meters) * 0.5, kFieldWidth.in(Meters) * 0.5 + Units.inchesToMeters(41.5));
      cages[1] = new Translation2d(kFieldLength.in(Meters) * 0.5, kFieldWidth.in(Meters) * 0.5 + Units.inchesToMeters(84.375));
      cages[2] = new Translation2d(kFieldLength.in(Meters) * 0.5, kFieldWidth.in(Meters) * 0.5 + Units.inchesToMeters(120.375));
      // spotless:on
    }
  }

  public static final class CoralStation {
    // spotless:off
    public static final Pose2d left = new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(291.20), Rotation2d.fromDegrees(306));
    public static final Pose2d right = new Pose2d(Units.inchesToMeters(33.51), Units.inchesToMeters(25.80), Rotation2d.fromDegrees(54));
    // spotless:on
  }

  public static final class Processor {
    public static final Pose2d center =
        new Pose2d(Units.inchesToMeters(235.73), 0, Rotation2d.fromDegrees(90));
  }

  /**
   * Returns the supplier of pose specified in the blue alliance origin corrected to the current
   * alliance.
   */
  public static Function<Pose2d, Pose2d> createFieldPoseSupplier() {
    return (Pose2d pose) -> {
      final var flipped =
          DriverStation.getAlliance().isPresent()
              && DriverStation.getAlliance().get() == Alliance.Red;

      return flipped ? ChoreoAllianceFlipUtil.flip(pose) : pose;
    };
  }

  public static Function<Pose2d, Pose2d> fieldPose = createFieldPoseSupplier();
}
