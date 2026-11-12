package frc.robot.lib;

import static edu.wpi.first.units.Units.Amps;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.units.measure.Current;

/**
 * SysId-specific SwerveRequest to characterize the translational characteristics of a swerve
 * drivetrain when using TorqueCurrentFOC mode.
 */
public class SysIdSwerveTranslationTorqueCurrent implements SwerveRequest {
  /** Torque current to apply to drive wheels */
  private double torqueCurrentToApply = 0;

  /** Local reference to a torque current request for the drive motors */
  private final TorqueCurrentFOC driveRequest = new TorqueCurrentFOC(0);

  /** Local reference to a position voltage request for the steer motors */
  private final PositionVoltage steerRequestVoltage = new PositionVoltage(0);

  public StatusCode apply(
      SwerveControlParameters parameters, SwerveModule<?, ?, ?>... modulesToApply) {
    for (int i = 0; i < modulesToApply.length; ++i) {
      modulesToApply[i].apply(
          driveRequest.withOutput(torqueCurrentToApply), steerRequestVoltage.withPosition(0));
    }
    return StatusCode.OK;
  }

  /**
   * Sets the torque current to apply to the drive wheels.
   *
   * @param amps Torque current to apply
   * @return this request
   */
  public SysIdSwerveTranslationTorqueCurrent withTorqueCurrent(double amps) {
    torqueCurrentToApply = amps;
    return this;
  }
  /**
   * Sets the torque current to apply to the drive wheels.
   *
   * @param value Torque current to apply
   * @return this request
   */
  public SysIdSwerveTranslationTorqueCurrent withTorqueCurrent(Current value) {
    torqueCurrentToApply = value.in(Amps);
    return this;
  }
}
