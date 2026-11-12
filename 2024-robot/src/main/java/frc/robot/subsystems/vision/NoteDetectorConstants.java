package frc.robot.subsystems.vision;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * Note detector subsystem related constants.
 *
 * <p>For camera constants see (Camera Module v1):
 * https://www.raspberrypi.com/documentation/accessories/camera.html
 */
public final class NoteDetectorConstants {
  /** Camera horizontal FOV */
  public static final double FOVh = Units.degreesToRadians(53.5);
  /** Camera vertical FOV */
  public static final double FOVv = Units.degreesToRadians(41.41);
  /** Camera frame width in pixels */
  public static final double FrameWidthPx = 640;
  /** Camera frame height in pixels */
  public static final double FrameHeightPx = 480;
  /** Camera sensor pixel size in mm */
  public static final double SensorPixelMm = 0.0014;
  /** Camera sensor width in pixels */
  public static final double SensorWidthPx = 2592;
  /** Camera sensor height in pixels */
  public static final double SensorHeightPx = 1944;
  /** Camera focal length in mm */
  public static final double FocalLengthMm = 3.6;
  /** Camera focal length in frame pixels */
  public static final double FocalLengthPx =
      (FocalLengthMm / SensorPixelMm) / (SensorWidthPx / FrameWidthPx);

  /** Camera height above ground in meters */
  public static final double CameraHeightMeters = Units.inchesToMeters(24);

  public static final double CameraHeightMetersSq = CameraHeightMeters * CameraHeightMeters;

  /** Camera translation from the COR in meters. X is front/rear, Y is left/right. */
  public static final Translation2d CameraTranslationMeters =
      new Translation2d(Units.inchesToMeters(5.5), 0.0);

  /** Note (game piece) width in meters */
  public static final double NoteWidthMeters = Units.inchesToMeters(14);

  /** Translation factor from normalized X coordinates to tan(yaw) */
  public static final double YawFactor = Math.tan(FOVh * 0.5);
}
