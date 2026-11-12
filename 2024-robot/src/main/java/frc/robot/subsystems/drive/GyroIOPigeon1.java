package frc.robot.subsystems.drive;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.PigeonIMU.PigeonState;
import com.ctre.phoenix.sensors.WPI_PigeonIMU;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.OptionalDouble;
import java.util.Queue;

/** Module IO implementation for Pigeon1 hooked via TalonSRX controller */
public class GyroIOPigeon1 implements GyroIO {
  private final WPI_PigeonIMU pigeon;
  private final Queue<Double> yawPositionQueue;
  private final double sign;

  public GyroIOPigeon1(int deviceNumber, boolean flipped) {
    pigeon = new WPI_PigeonIMU(deviceNumber);
    sign = flipped ? -1.0 : 1.0;
    pigeon.setYaw(0.0);
    yawPositionQueue =
        SparkMaxOdometryThread.getInstance()
            .registerSignal(
                () ->
                    pigeon.getLastError() == ErrorCode.OK
                        ? OptionalDouble.of(sign * pigeon.getYaw())
                        : OptionalDouble.empty());
  }

  @Override
  public void updateInputs(GyroIOInputs inputs) {
    inputs.connected = pigeon.getState() == PigeonState.Ready;
    inputs.yawPosition = Rotation2d.fromDegrees(sign * pigeon.getYaw());
    inputs.yawVelocityRadPerSec = Units.degreesToRadians(pigeon.getRate());

    inputs.odometryYawPositions =
        yawPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromDegrees(value))
            .toArray(Rotation2d[]::new);

    yawPositionQueue.clear();
  }
}
