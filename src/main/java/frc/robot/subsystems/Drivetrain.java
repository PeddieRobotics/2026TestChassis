package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.RobotMap;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain drivetrain;
    private final SwerveModule[] swerveModule;
    private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    private SwerveModuleState[] swerveModuleState;
    private final SwerveModulePosition[] swerveModulePosition;
    private final Pigeon2 gyro;
    private double heading;

    public Drivetrain() {
        // CANBus defaultCANBus = new CANBus(RobotMap.CANIVORE_NAME);
        CANBus defaultCANBus = new CANBus("rio");

        frontLeftModule = new SwerveModule(
            defaultCANBus,
            RobotMap.FRONT_LEFT_MODULE_DRIVE_ID,
            RobotMap.FRONT_LEFT_MODULE_TURN_ID,
            RobotMap.FRONT_LEFT_MODULE_CANCODER_ID,
            ModuleConstants.kFrontLeftModuleOffset
        );
        frontRightModule = new SwerveModule(
            defaultCANBus,
            RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID,
            RobotMap.FRONT_RIGHT_MODULE_TURN_ID,
            RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID,
            ModuleConstants.kFrontRightModuleOffset
        );
        backLeftModule = new SwerveModule(
            defaultCANBus,
            RobotMap.BACK_LEFT_MODULE_DRIVE_ID,
            RobotMap.BACK_LEFT_MODULE_TURN_ID,
            RobotMap.BACK_LEFT_MODULE_CANCODER_ID,
            ModuleConstants.kBackLeftModuleOffset
        );
        backRightModule = new SwerveModule(
            defaultCANBus,
            RobotMap.BACK_RIGHT_MODULE_DRIVE_ID,
            RobotMap.BACK_RIGHT_MODULE_TURN_ID,
            RobotMap.BACK_RIGHT_MODULE_CANCODER_ID,
            ModuleConstants.kBackRightModuleOffset
        );

        swerveModule = new SwerveModule[] { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
        swerveModulePosition = new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
        swerveModuleState = Constants.DriveConstants.kinematics.toSwerveModuleStates(
            new ChassisSpeeds(0, 0, 0)
        );

        gyro = new Pigeon2(RobotMap.GYRO_ID, defaultCANBus);
        setGyro(0);
    }
    
    public static Drivetrain getInstance() {
        if (drivetrain == null)
            drivetrain = new Drivetrain();
        return drivetrain;
    }
    
    public void setGyro(double yaw) {
        gyro.setYaw(yaw);
    }

    public void updateModulePositions() {
        for (int i = 0; i < swerveModulePosition.length; i++)
            swerveModulePosition[i] = swerveModule[i].getPosition();
    }

    public Rotation2d getHeadingRotation2d() {
        return gyro.getRotation2d();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerRotation) {
        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        ChassisSpeeds robotRelativeSpeeds;

        if (fieldOriented)
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeadingRotation2d());
        else
            robotRelativeSpeeds = fieldRelativeSpeeds;

        if (centerRotation == null)
            swerveModuleState = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        else
            swerveModuleState = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds, centerRotation);

        for (int i = 0; i > swerveModuleState.length; i++)
            swerveModuleState[i].optimize(new Rotation2d(swerveModule[i].getCANcoderReading()));

        setModuleStates(swerveModuleState);
    }

    public void setModuleStates(SwerveModuleState[] statesList) {
        for (int i = 0; i < statesList.length; i++) {
            swerveModule[i].setOptimizedState(statesList[i]);
        }
    }

    public double getHeading() {
        heading = gyro.getRotation2d().getDegrees();
        return Math.IEEEremainder(heading, 360);
    }

    public void lockModules() {
        frontLeftModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4)));
        backLeftModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)));
        frontRightModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)));
        backRightModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(-3 * Math.PI / 4)));
    }
}
