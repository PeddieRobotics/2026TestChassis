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
        public static final double kTurretToHubHeight = 0.0;
        public static final double kTurretDisplacementX = 0.225; //m
        public static final double kTurretDisplacementY = 0.0;

        // PID values for the turret pitch
        public static final double kTurretPositionS = 0;
        public static final double kTurretPositionV = 0; 
        public static final double kTurretPositionA = 0; 
        public static final double kTurretPositionP = 0; //overcome static friction
        public static final double kTurretPositionI = 0; 
        public static final double kTurretPositionD = 0; 
        public static final double kTurretPositionF = 0; 

        // PID values for the turret yaw (Motion magic torque current FOC)
        public static final double kTurretYawS = 0;
        public static final double kTurretYawV = 0; 
        public static final double kTurretYawA = 0; 
        public static final double kTurretYawP = 250; // overcome static friction
        public static final double kTurretYawI = 0; 
        public static final double kTurretYawD = 12; 
        public static final double kTurretYawF = 0; 
        public static final double kCruiseVelocity = 4;
        public static final double kMaxAcceleration = 10; // radians/seconds^2

        // valid range to not destroy turret is [-kTurretRange, kTurretRange]
        public static final double kTurretRange = 270;

        public static final int kKrakenGearTeeth = 12;
        public static final int kTurretGearTeeth = 200;
        public static final int kEncoderGear1Teeth = 20; // n1
        public static final int kEncoderGear2Teeth = 21; // n2

        public static final double kKrakenToTurretRatio = (double)kTurretGearTeeth/kKrakenGearTeeth;

        public static final int kTurretMotorDeviceId = 60;
        public static final int kEncoderId1 = 61;
        public static final int kEncoderId2 = 62;
        
        // set to true when zeroing turret
        public static final boolean ZEROING_MODE = false;
        
        // WHEN SETTING THE MAGNET OFFSET, DO NOT MAKE THESE ZERO
        // SET ZEROING_MODE TO TRUE, VALUES HERE DON'T MATTER
        // negative of "raw absolute position", don't add mod amount offset
        public static final double kEncoder1MagnetOffset = -0.942626953125;
        public static final double kEncoder2MagnetOffset = -0.96044921875;

        // this is the "number of gears moved" (found by CRT)
        // where the turret's position is 0 degrees, aka forward
        // 210
        public static final int kZeroPositionTeethRaw = kEncoderGear1Teeth * kEncoderGear2Teeth / 2;

        // 1 degree = (kTurretGearTeeth / 360) teeth
        public static double positionDegreeToTeeth(double degree) {
            return degree * kTurretGearTeeth / 360;
        }

        // 1 tooth = (360 / kTurretGearTeeth) degrees
        public static double positionTeethToDegree(double teeth) {
            return teeth * 360 / kTurretGearTeeth;
        }

        public static final double kP = 0.0;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kS = 0.0;
        public static final double kFF = 0.0;

        public static final double kEpsilon = 1.0;
        public static final double kVoltageMax = 2;
        
        // these are always positive, see Turret.java for explanation
        public static final double kMinPositionRotations = -0.75;
        public static final double kMaxPositionRotations = 0.75;
        
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


        public static final double kWheelDiameterIn = 2 * (1.9367026923287955);
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

        public static final double kSteerS = 0.16;
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
        public static final double kBackLeftMagnetOffset =0;
        public static final double kBackRightMagnetOffset =0;
        public static final double kFrontLeftMagnetOffset = 0;
        public static final double kFrontRightMagnetOffset =0;

        public static final double kMaxModuleSpeed = 4.4;
        public static final double kSkidThreshold = 0;

        public static final double kTrackWidth = Units.inchesToMeters(22.5);
        public static final double kWheelBase = Units.inchesToMeters(22.5);

        public static final Translation2d[] kSwerveModuleLocations = {
                new Translation2d(kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(kWheelBase / 2.0, -kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, kTrackWidth / 2.0),
                new Translation2d(-kWheelBase / 2.0, -kTrackWidth / 2.0),
        };

        public static final SwerveDriveKinematics kKinematics = new SwerveDriveKinematics(
                kSwerveModuleLocations[0],
                kSwerveModuleLocations[1],
                kSwerveModuleLocations[2],
                kSwerveModuleLocations[3]
        );

        public static final SwerveDriveKinematics kSkidKinematics = new SwerveDriveKinematics(
                kSwerveModuleLocations[0],
                kSwerveModuleLocations[1],
                kSwerveModuleLocations[2],
                kSwerveModuleLocations[3]
        );

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
    public static final class FieldConstants {
            
            public static final double kBlueHubX = 4.63;
            public static final double kBlueHubY = 8.07;

            public static final double kRedHubX = 16.54 - kBlueHubX;
            public static final double kRedHubY = 8.07 - kBlueHubY;

            public static final Translation2d kBlueHub = new Translation2d(kBlueHubX, kBlueHubY);
            public static final Translation2d kRedHub = new Translation2d(kRedHubX, kRedHubY);
            
            public static Translation2d getHub() {
                return (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) ? kBlueHub : kRedHub;
            }
            
            public static final double bluePassPositionX = 0;
            public static final double bluePassPositionY = 0;
            public static final double redPassPositionX = 0;
            public static final double redPassPositionY = 0;

            public static final double kTransitionShiftEnd = 10;       
            public static final double kShift1End = 35;
            public static final double kShift2End = 60;
            public static final double kShift3End = 85;
            public static final double kShift4End = 110;
            public static final double kHoldThreshold = 7;

            public static final double bottomLeftCornerX = 1; //can tune this later
            public static final double bottomLeftCornerY = 1;
            public static final double bottomRightCornerX = 15; 
            public static final double bottomRightCornerY = 1;
            public static final double topLeftCornerX = 1; 
            public static final double topLeftCornerY = 7;
            public static final double topRightCornerX = 15; 
            public static final double topRightCornerY = 7;
        }
        public static class HopperConstants{

        public static final double kGroundIntakeHopperSpeed = 0;
        public static final double kShootHopperSpeed = 0;
        public static final double kOuttakeHopperSpeed = 0;
        public static final int kHPIntakeHopperSpeed = 0;
        public static final double kFeedFlywheelLayupSpeed = 0;
        public static final double kFeedFlywheel = 0;
        public static final double kFeedFlywheelPassSpeed = 0;

    }
    public static class IntakeConstants{

        public static final double kIntakeSpeed = 0.0;
        public static final double kIntakeFeedSpeed = 0.0;
        public static double kHopperCurrentLimit;
    }
    public static class HoodConstants {
        public static final double kHoodTolerance = 0.0;
        public static final double kHoodCPR = 4096;
        public static final double kHoodMaxAngle = 85.0;
        public static final double kHoodMinAngle = 30.0;
        public static final double kHoodPositionKp = 0;
        public static final double kHoodPositionKd = 0;
        public static final double kHoodPassingAngle = 0;
    }
    public static class ShooterConstants{
        // constant velocity and position for passing
        public static final double kPassSpeed = 0.0;
        /*Physics polynomial coefficients
        example ax^3+bx^2y+cxy^2+dy^3
        */
        //this is for the polynomial for velocity
        public static final double aVelocity = 0; // the first coefficient for the physics polynomial for shooter velocity
        public static final double bVelocity = 0;
        public static final double cVelocity = 0;
        public static final double dVelocity = 0;

        // this is for the polynomial for pitch angle
        public static final double aPitch = 0; // the first coefficient for the physics polynomial for pitch angle
        public static final double bPitch = 0;
        public static final double cPitch = 0;
        public static final double dPitch = 0;
        
        // coefficients for func to calculate time in air--> integral of distance travelled as time increases (by dx ~ 0.001, from t=0)
        public static final double aTimeInAir = 0;
        public static final double bTimeInAir = 0;
        public static final double cTimeInAir = 0;
        public static final double dTimeInAir = 0;
        
        public static final double kShooterSpeedS = 0;
        public static final double kShooterSpeedV = 0; 
        public static final double kShooterSpeedA = 0; 
        public static final double kShooterSpeedP = 0; //overcome static friction
        public static final double kShooterSpeedI = 0; 
        public static final double kShooterSpeedD = 0; 
        public static final double kShooterSpeedF = 0;
    }
    public static final class ScoreConstants {
        // TODO: make
    }
    public static final class ClimberConstants {

        public static final int kClimberRetractedPosition = 0;
        public static final int kClimberDeployedPercentOutput =0;
        public static final double kClimberDeployedPosition = 0;
    }
    public static final class CameraConstants {
        public static final String kFrontCamName = "limelight-front";
        public static final String kBackCamName = "limelight-back";
        public static final String kLeftCamName = "limelight-left";
        public static final String kRightCamName = "limelight-right";

        public static final int kNumberLimelights = 4;
    }
}