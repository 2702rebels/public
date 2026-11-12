package frc.robot.util;

/** A value with attached timestamp in seconds. */
public record TimestampedValue<T>(double timestamp, T value) {}
