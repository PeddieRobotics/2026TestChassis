// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.utils;

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

    public static class TurretConstants {
        public static final int kTurretMotorDeviceId = 0;
        public static final int kEncoderId1 = 0;
        public static final int kEncoderId2 = 0;
        
        // valid range to not destroy turret is [-kTurretRange, kTurretRange]
        public static final double kTurretRange = 270;

        public static final int kTurretGearTeeth = 200;
        public static final int kEncoderGear1Teeth = 20; // n1
        public static final int kEncoderGear2Teeth = 21; // n2

        // this is the "number of gears moved" (found by CRT)
        // where the turret's position is 0 degrees, aka forward
        // 210
        public static final int kZeroPositionTeeth = kEncoderGear1Teeth * kEncoderGear2Teeth / 2;

        // 1 degree = (kTurretGearTeeth / 360) teeth
        public static double positionDegreeToTeeth(double degree) {
            return degree * kTurretGearTeeth / 360;
        }

        // 1 tooth = (360 / kTurretGearTeeth) degrees
        public static double positionTeethToDegree(double teeth) {
            return teeth * 360 / kTurretGearTeeth;
        }

        public static final double kTurretP = 0.0;
        public static final double kTurretI = 0.0;
        public static final double kTurretD = 0.0;
        public static final double kTurretFF = 0.0;
        
        // these are always positive, see Turret.java for explanation
        public static final double kMinPositionTeethRaw = positionDegreeToTeeth(-kTurretRange) + kZeroPositionTeeth;
        public static final double kMaxPositionTeethRaw = positionDegreeToTeeth(kTurretRange) + kZeroPositionTeeth;
        
        public static class CRTConstants {
            public static final int y_1 = 1; 
            public static final int y_2 = -1; 
            
            // by definition from CRT: y_1 is such that y_1 * m_1 ≡ 1 (mod n_1)
            // from the CRT: m_1 = n_2 = 21, m_2 = n_1 = 20
            // y_1 * m_1 = 1 * 21 = 21 ≡ 1 (mod 20)
            // y_2 * m_2 = -1 * 20 = -20 ≡ 1 (mod 21)
        }
    }

    public static class ModuleConstants {
        public static final double kDriveMotorCurrentLimit = 40.0;
        public static final double kTurnMotorCurrentLimit = 40.0;
        public static final double kWheelDiameterIn = 4.0;
        public static final double kDriveMotorReduction = 7.13; // placeholder

        public static final double kDriveEncoderVelocityFactor = (Math.PI * Units.inchesToMeters(kWheelDiameterIn)
                / kDriveMotorReduction);
        public static final double kSteerMotorReduction = 18.75;

        public static final double kDriveS = 0.28;
        public static final double kDriveV = 0.1;
        public static final double kDriveA = 0.0;
        public static final double kDriveP = 0.3;
        public static final double kDriveI = 0.0;
        public static final double kDriveD = 0.0;
        public static final double kDriveFF = 0.0;

        public static final double kSteerS = 0.2;
        public static final double kSteerV = 0.0;
        public static final double kSteerA = 0.0;
        public static final double kSteerP = 135.0;
        public static final double kSteerI = 0.0;
        public static final double kSteerD = 7.0;
        public static final double kSteerFF = 0.0;

        public static final double kFrontLeftModuleOffset = -0.651367;
        public static final double kFrontRightModuleOffset = 0.145996;
        public static final double kBackLeftModuleOffset = 0.020508;
        public static final double kBackRightModuleOffset = -0.222900;
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