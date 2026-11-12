package frc.robot.subsystems.drive;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import java.util.OptionalDouble;
import java.util.Queue;

/** Module IO implementation for Pigeon2 */
public class GyroIOPigeon2 implements GyroIO {
  private final Pigeon2 pigeon;
  private final StatusSignal<Double> yaw;
  private final Queue<Double> yawPositionQueue;
  private final StatusSignal<Double> yawVelocity;

  public GyroIOPigeon2(int deviceNumber, boolean phoenixDrive) {
    pigeon = new Pigeon2(deviceNumber);
    pigeon.getConfigurator().apply(new Pigeon2Configuration());
    pigeon.getConfigurator().setYaw(0.0);

    yaw = pigeon.getYaw();
    yaw.setUpdateFrequency(Constants.odometryFrequency);

    yawVelocity = pigeon.getAngularVelocityZWorld();
    yawVelocity.setUpdateFrequency(100.0);

    pigeon.optimizeBusUtilization();

    if (phoenixDrive) {
      yawPositionQueue = PhoenixOdometryThread.getInstance().registerSignal(pigeon, yaw);
    } else {
      yawPositionQueue =
          SparkMaxOdometryThread.getInstance()
              .registerSignal(
                  () ->
                      yaw.refresh().getStatus().isOK()
                          ? OptionalDouble.of(yaw.getValueAsDouble())
                          : OptionalDouble.empty());
    }
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = BaseStatusSignal.refreshAll(yaw, yawVelocity).equals(StatusCode.OK);
    inputs.yawPosition = Rotation2d.fromDegrees(yaw.getValueAsDouble());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(yawVelocity.getValueAsDouble());

    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    yawPositionQueue.clear();
  }
}
