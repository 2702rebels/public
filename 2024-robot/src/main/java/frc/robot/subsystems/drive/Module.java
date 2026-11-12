package frc.robot.subsystems.drive;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.Misc;
import org.littletonrobotics.junction.Logger;

public class Module {
  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;
  private final Rotation2d angularOffset;
  private final SwerveModuleState moduleState;
  private final SwerveModulePosition modulePosition;

  // spotless:off
  private static final LoggedTunableNumber driveP  = new LoggedTunableNumber("Drive/Module/Drive P");
  private static final LoggedTunableNumber driveI  = new LoggedTunableNumber("Drive/Module/Drive I");
  private static final LoggedTunableNumber driveD  = new LoggedTunableNumber("Drive/Module/Drive D");
  private static final LoggedTunableNumber driveFF = new LoggedTunableNumber("Drive/Module/Drive FF");
  private static final LoggedTunableNumber driveS  = new LoggedTunableNumber("Drive/Module/Drive kS");
  private static final LoggedTunableNumber driveV  = new LoggedTunableNumber("Drive/Module/Drive kV");
  private static final LoggedTunableNumber driveA  = new LoggedTunableNumber("Drive/Module/Drive kA");

  private static final LoggedTunableNumber turnP   = new LoggedTunableNumber("Drive/Module/Turn P");
  private static final LoggedTunableNumber turnI   = new LoggedTunableNumber("Drive/Module/Turn I");
  private static final LoggedTunableNumber turnD   = new LoggedTunableNumber("Drive/Module/Turn D");
  private static final LoggedTunableNumber turnFF  = new LoggedTunableNumber("Drive/Module/Turn FF");
  // spotless:on

  static {
    driveP.initDefault(DriveConstants.moduleFactors.driveMotor().kP());
    driveI.initDefault(DriveConstants.moduleFactors.driveMotor().kI());
    driveD.initDefault(DriveConstants.moduleFactors.driveMotor().kD());
    driveFF.initDefault(DriveConstants.moduleFactors.driveMotor().kFF());
    driveS.initDefault(DriveConstants.moduleFactors.driveMotorFF().kS());
    driveV.initDefault(DriveConstants.moduleFactors.driveMotorFF().kV());
    driveA.initDefault(DriveConstants.moduleFactors.driveMotorFF().kA());
    turnP.initDefault(DriveConstants.moduleFactors.turnMotor().kP());
    turnI.initDefault(DriveConstants.moduleFactors.turnMotor().kI());
    turnD.initDefault(DriveConstants.moduleFactors.turnMotor().kD());
    turnFF.initDefault(DriveConstants.moduleFactors.turnMotor().kFF());
  }

  private SimpleMotorFeedforward driveMotorFF =
      new SimpleMotorFeedforward(
          DriveConstants.moduleFactors.driveMotorFF().kS(),
          DriveConstants.moduleFactors.driveMotorFF().kV(),
          DriveConstants.moduleFactors.driveMotorFF().kA());

  public Module(ModuleIO io, int index, Rotation2d chassisAngularOffset) {
    this.io = io;
    this.index = index;
    angularOffset = chassisAngularOffset;
    moduleState = new SwerveModuleState();
    modulePosition = new SwerveModulePosition();

    setBrakeMode(true);
  }

  /**
   * Updates inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);

    // update current module state and position
    // note that we do not reallocate the state and position placeholders,
    // instead we update them here in place, once new inputs are processed
    //
    // since this method is invoked at the main loop robot frequency,
    // there is no need to sample drive velocity more frequently, as
    // odometry tracking does not depend on velocity, but only on position
    //
    // module angle is already adjusted by the raw offset, reporting values
    // with respect to the robot's chassis

    moduleState.speedMetersPerSecond = inputs.driveVelocityMetersPerSec;
    moduleState.angle = inputs.turnPosition;
    modulePosition.distanceMeters = inputs.drivePositionMeters;
    modulePosition.angle = inputs.turnPosition;
  }

  public void periodic() {
    final var callerId = hashCode();

    // update controllers if tunable numbers have changed
    if (driveP.hasChanged(callerId)
        || driveI.hasChanged(callerId)
        || driveD.hasChanged(callerId)
        || driveFF.hasChanged(callerId)) {
      io.configureDrivePID(driveP.get(), driveI.get(), driveD.get(), driveFF.get());
    }

    if (driveS.hasChanged(callerId) || driveV.hasChanged(callerId) || driveA.hasChanged(callerId)) {
      driveMotorFF = new SimpleMotorFeedforward(driveS.get(), driveV.get(), driveA.get());
    }

    if (turnP.hasChanged(callerId)
        || turnI.hasChanged(callerId)
        || turnD.hasChanged(callerId)
        || turnFF.hasChanged(callerId)) {
      io.configureTurnPID(turnP.get(), turnI.get(), turnD.get(), turnFF.get());
    }
  }

  /**
   * Runs the module with the specified setpoint state and an optional angular velocity. Returns the
   * optimized state.
   */
  public void runSetpoint(SwerveModuleState state, double omegaRadiansPerSecond) {
    // optimize state based on current angle
    //
    // we do not use FRC WPILIB SwerveModuleState.optimize method,
    // instead we use similar logic that reduces allocations, note
    // that we also adjust the angle by the module angular offset
    var desiredAngle = Misc.clampAngle(state.angle.getRadians() + angularOffset.getRadians());
    var currentAngle = Misc.clampAngle(getRawAngle().getRadians());
    var desiredSpeed = state.speedMetersPerSecond;

    // 91 degrees to avoid instability
    if (Math.abs(desiredAngle - currentAngle) > 1.58825) {
      desiredSpeed *= -1;
      desiredAngle = Misc.clampAngle(desiredAngle + Math.PI);
    }

    // command driving and turning motors towards their respective setpoints
    io.setDriveVelocity(desiredSpeed, driveMotorFF.calculate(desiredSpeed));

    final var turningFF =
        DriveConstants.drivetrainConfig.turnMotor().nominalVoltage()
            * omegaRadiansPerSecond
            / DriveConstants.drivetrainConfig.turnMotorMaxAngularSpeed();

    io.setTurnAngle(desiredAngle, turningFF);
  }

  /** Runs characterization with the specified voltage */
  public void runCharacterization(double volts, double turnAngleRadians) {
    io.setTurnAngle(Misc.clampAngle(turnAngleRadians + angularOffset.getRadians()));
    io.setDriveVoltage(volts);
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the recently sampled module positions. */
  public double[] getSampledPositionsMeters() {
    return inputs.odometryDrivePositionsMeters;
  }

  /** Returns the recently sampled module turn angles. */
  public Rotation2d[] getSampledAngles() {
    return inputs.odometryTurnPositions;
  }

  /** Returns the current turn angle of the module (already adjusted for offset). */
  public Rotation2d getAngle() {
    return inputs.turnPosition;
  }

  /** Returns the current raw (unadjusted) turn angle of the module. */
  public Rotation2d getRawAngle() {
    return Misc.add(inputs.turnPosition, angularOffset);
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionMeters;
  }

  /** Returns the current drive velocity of the module in meters/sec. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityMetersPerSec;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return modulePosition;
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return moduleState;
  }

  /** Resets encoders position. */
  public void resetEncoders() {
    io.resetEncoders();
  }
}
