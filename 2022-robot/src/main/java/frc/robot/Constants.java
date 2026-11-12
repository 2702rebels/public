// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/** Add your docs here. */
public final class Constants {
    public static final class RobotConstants {
        public static final double kRobotPeriod = 0.01;
    }

    public static final class DrivetrainConstants {
        public static final int kLeftMotorLeaderCanId = 1;
        public static final int kLeftMotorFollowerCanId = 2;
        public static final int kRightMotorLeaderCanId = 3;
        public static final int kRightMotorFollowerCanId = 4;

        public static final int kShifterForwardsPcmId = 0;
        public static final int kShifterReversePcmId = 1;
    }

    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static final class CompressorConstants {
        public static final int kCompressorMinPressure = 70;
        public static final int kCompressorMaxPressure = 120;
    }
}
