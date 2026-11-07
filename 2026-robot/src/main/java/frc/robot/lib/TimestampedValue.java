package frc.robot.lib;

/** A value with attached timestamp in microseconds. */
public final record TimestampedValue<T>(long timestamp, T value) {}
