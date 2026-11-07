package frc.robot.lib;

import edu.wpi.first.math.geometry.Translation2d;

/** A Translation2d with attached timestamp in microseconds. */
public final record TimestampedTranslation2d(long timestamp, Translation2d value) {}
