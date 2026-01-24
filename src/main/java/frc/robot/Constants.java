// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }

    public static class ModuleConstants {
        public static final double kDriveMotorCurrentLimit = 40.0;
        public static final double kTurnMotorCurrentLimit = 40.0;
        public static final double kWheelDiameterIn = 4.0;
        public static final double kDriveMotorReduction = 7.13; // placeholder

        public static final double kDriveEncoderVelocityFactor = (Math.PI * Units.inchesToMeters(kWheelDiameterIn)
                / kDriveMotorReduction);
        public static final double kSteerMotorReduction = 18.75;

        public static final double kDrivingS = 0.28;
        public static final double kDrivingV = 0.1;
        public static final double kDrivingA = 0.0;
        public static final double kDrivingP = 0.3;
        public static final double kDrivingI = 0.0;
        public static final double kDrivingD = 0.0;
        public static final double kDrivingFF = 0.0;

        public static final double kSteerS = 0.2;
        public static final double kSteerV = 0.0;
        public static final double kSteerA = 0.0;
        public static final double kSteerP = 135.0;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 7.0;
        public static final double kSteerFF = 0.0;

        public static final double kFrontLeftModuleOffset = -0.651367 ;
        public static final double kFrontRightModuleOffset = 0.145996 ;
        public static final double kBackLeftModuleOffset = 0.020508;
        public static final double kBackRightModuleOffset = -0.222900 ;
    }

    public static class DriveConstants {
        public static final double trackWidth = 2.0;
        public static final double wheelBase = 2.0;
        public static final Translation2d[] swerveModuleLocations = {
            new Translation2d(wheelBase / 2, trackWidth / 2),
            new Translation2d(wheelBase / 2, -trackWidth / 2),
            new Translation2d(-wheelBase / 2, trackWidth / 2),
            new Translation2d(-wheelBase / 2, -trackWidth / 2)
        };

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            swerveModuleLocations[0],
            swerveModuleLocations[1],
            swerveModuleLocations[2],
            swerveModuleLocations[3]
        );

        public static final double kMaxRotationSpeed = 0.5 * Math.PI;
        public static final double kMaxFloorSpeed = 4;
    }
}