package frc.robot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearAcceleration;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.units.measure.Time;

public final class Constants {
  /** Deployment mode */
  public static enum Deployment {
    /** Development mode, all tunable values are available and extended logging is enabled */
    kDevelopment,
    /** Compeition mode, only critical tunable values are available and logging is reduced */
    kCompetition,
  }

  /**
   * Target deployment mode. This should be set to kCompetition when compiling code for competition
   * use to reduce the performance impact of the telemetry.
   */
  public static final Deployment DeploymentMode = Deployment.kCompetition;

  public static final class Settings {
    /** RIO Logging */
    public static final boolean rioLoggingEnabled = true;

    /** Epilogue Logging */
    public static final boolean epilogueLoggingEnabled = true;

    /** CTRE Logging */
    public static final boolean ctreLoggingEnabled = false;

    /** Logging timing stats */
    public static final boolean statsEnabled = true;

    /** SysId bindings */
    public static final boolean sysIdBindings = false;

    /** Driving disabled */
    public static final boolean drivingDisabled = false;
  }

  /** NT prefix for tunable keys */
  public static final String tunablePrefix = "/Tunable";

  /** Robot main loop cycle period in seconds */
  public static final double cyclePeriodSeconds = 0.02;

  public static final class Network {
    public static final int UDP_PORT_TIMESYNC = 27020;
    public static final int UDP_PORT_APRILTAG_DETECTOR = 27022;
  }

  /** RoboRIO CAN identifiers */
  public static final class CanID {
    public static final int CANDUIT = 4;
  }

  /** CANivore identifiers */
  public static final class CanivoreID {
    public static final int MOTOR_SHOOTER_FLYWHEEL_LEADER = 37;
    public static final int MOTOR_SHOOTER_FLYWHEEL_FOLLOWER = 38;
    public static final int MOTOR_SHOOTER_HOOD = 39;
    public static final int MOTOR_TURRET = 36;
    public static final int MOTOR_HOPPER_SPINDEXER = 32;
    public static final int MOTOR_HOPPER_FEED = 35;
    public static final int MOTOR_INTAKE_ROLLER_LEADER = 31;
    public static final int MOTOR_INTAKE_ROLLER_FOLLOWER = 41;
    public static final int MOTOR_INTAKE_DEPLOY = 33;
    public static final int MOTOR_CLIMB = 34;
  }

  /** CANduit GPIO identifiers */
  public static final class CANduitID {
    public static final int TURRET_ENCODER1 = 3;
    public static final int TURRET_ENCODER2 = 2;
    public static final int CLIMB_SENSOR = 4;
  }

  /** RoboRIO DIO identifiers */
  public static final class DioID {}

  /** RoboRIO PWM identifiers */
  public static final class PwmID {
    public static final int LED = 0;
  }

  public static final class RevConstants {
    /**
     * REV NEO550 motor constants
     * https://docs.revrobotics.com/brushless/neo/550#motor-specifications
     */
    public static final class NEO550 {
      public static final double kV = 917;
      public static final double freeSpeed = 11000;
    }

    /**
     * REV NEO v1 motor constants
     * https://docs.revrobotics.com/brushless/neo/v1.1/neo-v1#motor-specifications
     */
    public static final class NEOv1 {
      public static final double kV = 473;
      public static final double freeSpeed = 5676;
    }
  }

  public static final class DriveConstants {
    /** Maximum linear speed */
    public static final LinearVelocity maxSpeed = MetersPerSecond.of(4);

    /** Maximum acceleration time */
    public static final Time maxAccelerationTime = Seconds.of(0.5);

    /** Maximum linear acceleration */
    public static final LinearAcceleration maxAcceleration = maxSpeed.div(maxAccelerationTime);

    /** Maximum angular velocity */
    public static final AngularVelocity maxAngularRate = RotationsPerSecond.of(1);

    /** Maximum angular acceleration */
    public static final AngularAcceleration maxAngularAcceleration =
        maxAngularRate.div(maxAccelerationTime);

    /** Translation tolerance */
    public static final double kTranslationTolerance = 0.05;

    /** Rotation tolerance */
    public static final double kRotationTolerance = Units.degreesToRadians(3);

    /** PID configuration for cross-track controllers */
    public static final class CrossTrack {
      public static final double kP = 1;
      public static final double kI = 0;
      public static final double kD = 0;
    }

    /** PID configuration for translation controllers */
    public static final class Translation {
      public static final double kP = 3;
      public static final double kI = 0;
      public static final double kD = 0;
    }

    /** PID configuration for rotation controllers */
    public static final class Rotation {
      public static final double kP = 2;
      public static final double kI = 0;
      public static final double kD = 0;
    }
  }

  public static final class LedConstants {
    /* Total length of LED strip (number of addressable LEDs) */
    public static final int kLength = 39;

    /** Individual segments */
    public static final int[] kSegments = {2, 7, 7, 7, 7, 7, 2};
  }
}
