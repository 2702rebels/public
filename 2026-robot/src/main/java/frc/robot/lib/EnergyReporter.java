package frc.robot.lib;

/** Represents a subsystem that reports its energy usage. */
public interface EnergyReporter {
  /** Returns total supply current across all devices in Amps. */
  double getTotalSupplyCurrent();
}
