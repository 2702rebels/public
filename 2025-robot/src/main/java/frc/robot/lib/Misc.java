package frc.robot.lib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.util.Color;

public final class Misc {
  private static final double TwoPI = 2 * Math.PI;

  /** Default numerical threshold for "zero" values */
  public static final double Epsilon = 1e-6;

  /** Determine whether two numbers are equal within the epsilon neighborhood. */
  public static boolean epsilonEquals(double a, double b, double epsilon) {
    final var d = a - b;
    return -epsilon <= d && d <= epsilon;
  }

  /** Determine whether two numbers are equal within the default epsilon neighborhood. */
  public static boolean epsilonEquals(double a, double b) {
    return epsilonEquals(a, b, Misc.Epsilon);
  }

  /** Adds angles with minimum allocations. */
  public static Rotation2d add(Rotation2d a, Rotation2d b) {
    return new Rotation2d(a.getRadians() + b.getRadians());
  }

  /** Subtracts angles with minimum allocations. */
  public static Rotation2d subtract(Rotation2d a, Rotation2d b) {
    return new Rotation2d(a.getRadians() - b.getRadians());
  }

  /** Returns angle in radians clamped to (-PI, PI] range. */
  public static Rotation2d clampAngle(Rotation2d angle) {
    return Rotation2d.fromRadians(clampAngle(angle.getRadians()));
  }

  /** Returns angle in radians clamped to (-PI, PI] range. */
  public static double clampAngle(double angleRadians) {
    while (angleRadians <= -Math.PI) {
      angleRadians += TwoPI;
    }

    while (angleRadians > Math.PI) {
      angleRadians -= TwoPI;
    }

    return angleRadians;
  }

  /** Determines if the angle in radians is in the specified sector. */
  public static boolean betweenAngle(double angle, double startAngle, double endAngle) {
    final var v = clampAngle(angle);

    // CCW arcs
    return startAngle > endAngle
        ? v >= startAngle || v <= endAngle
        : v >= startAngle && v <= endAngle;
  }

  /**
   * Returns angle in radians representing the shortest arc between two inputs. The sign of returned
   * value indicates the direction of the rotation: positive is CCW, negative is CW. For example:
   *
   * <pre>
   * from -> to = delta
   * 30° -> -30° = -60°
   * 30° -> 150° = 120°
   * 30° -> 300° = -90°
   * -150° -> 150° = -60°
   * -30° -> -270° = 120°
   * </pre>
   */
  public static double shortestArc(double fromRadians, double toRadians) {
    return clampAngle(toRadians - fromRadians);
  }

  /** WPILIB HSV: Hue [0-180), Saturation [0-255], Value [0-255] */
  public record ColorHSV(int hue, int saturation, int value) {}

  /** Returns WPILIB HSV representation of the RGB color. */
  public static ColorHSV toHSV(Color color) {
    final var r = color.red; // 0-1
    final var g = color.green; // 0-1
    final var b = color.blue; // 0-1

    final var max = Math.max(Math.max(r, g), b);
    final var min = Math.min(Math.min(r, g), b);
    final var c = max - min;

    final var v = max;
    final var s = v < Misc.Epsilon ? 0 : (c / v);

    var h = -1.0;
    if (c < Misc.Epsilon) {
      h = 0;
    } else if (max == r) {
      h = (60 * ((g - b) / c) + 360) % 360;
    } else if (max == g) {
      h = (60 * ((b - r) / c) + 120) % 360;
    } else if (max == b) {
      h = (60 * ((r - g) / c) + 240) % 360;
    }

    // WPILIB uses non-standard ranges for HSV
    // H = [0-180) instead of [0-360)
    // S = [0-255] instead of [0-100]
    // V = [0-255] instead of [0-100]
    return new ColorHSV(
        (int) Math.round(h / 2), (int) Math.round(s * 255), (int) Math.round(v * 255));
  }
}
