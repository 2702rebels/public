package frc.robot.lib;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.RobotController;
import java.util.LinkedHashMap;
import java.util.Map;

public class EnergyTracker {
  private static EnergyTracker instance;

  public static synchronized EnergyTracker getInstance() {
    if (instance == null) instance = new EnergyTracker();
    return instance;
  }

  private record RegisteredReporter(
      EnergyReporter reporter,
      DoublePublisher publisherCurrent,
      DoublePublisher publisherPower,
      DoublePublisher publisherEnergy) {}

  private final Map<String, RegisteredReporter> reporters = new LinkedHashMap<>();
  private final Map<String, Double> subsystemEnergy = new LinkedHashMap<>();
  private long prevTimestamp;

  private double totalCurrent = 0.0;
  private double totalPower = 0.0;
  private double totalEnergy = 0.0;

  private static final NetworkTable table = NetworkTableInstance.getDefault().getTable("Energy");
  private final DoublePublisher totalCurrentTopic = table.getDoubleTopic("TotalCurrent").publish();
  private final DoublePublisher totalPowerTopic = table.getDoubleTopic("TotalPower").publish();
  private final DoublePublisher totalEnergyTopic = table.getDoubleTopic("TotalEnergy").publish();

  private static final double kCurrentPigeon = 0.04;
  private static final double kCurrentCANivore = 0.03;
  private static final double kCurrentRadio = 0.5;
  private static final double kCurrentJetson = 1;

  private static final double kCurrentOther =
      kCurrentRadio + kCurrentPigeon + kCurrentCANivore + kCurrentJetson * 2;

  private static double joulesToWattHours(double joules) {
    return joules / 3600.0;
  }

  public void register(String name, EnergyReporter reporter) {
    final var subtable = table.getSubTable(name);
    reporters.put(
        name,
        new RegisteredReporter(
            reporter,
            subtable.getDoubleTopic("Current").publish(),
            subtable.getDoubleTopic("Power").publish(),
            subtable.getDoubleTopic("Energy").publish()));

    subsystemEnergy.put(name, 0.0);
  }

  private void accountFor(double batteryVoltage, double dt, double current) {
    final var power = current * batteryVoltage;
    final var energy = joulesToWattHours(power * dt);

    totalCurrent += current;
    totalPower += power;
    totalEnergy += energy;
  }

  public void periodic() {
    // reset current and power counters
    totalCurrent = 0.0;
    totalPower = 0.0;

    final var timestamp = RobotController.getTime();
    final var dt = (double) (timestamp - prevTimestamp) / 1e6; // microseconds -> seconds
    prevTimestamp = timestamp;

    final var batteryVoltage = RobotController.getBatteryVoltage();

    // account for controller consumption and other devices with approx. static current draw
    accountFor(batteryVoltage, dt, RobotController.getInputCurrent());
    accountFor(batteryVoltage, dt, kCurrentOther);

    for (var entry : reporters.entrySet()) {
      final var r = entry.getValue();
      final var reporter = r.reporter();
      final var current = reporter.getTotalSupplyCurrent();
      final var power = current * batteryVoltage;
      final var energy = joulesToWattHours(power * dt);

      final var k = entry.getKey();
      final var accumulatedEnergy = subsystemEnergy.get(k) + energy;
      subsystemEnergy.put(k, accumulatedEnergy);

      r.publisherCurrent.set(current);
      r.publisherPower.set(power);
      r.publisherEnergy.set(accumulatedEnergy);

      totalCurrent += current;
      totalPower += power;
      totalEnergy += energy;
    }

    totalCurrentTopic.set(totalCurrent);
    totalPowerTopic.set(totalPower);
    totalEnergyTopic.set(totalEnergy);
  }
}
