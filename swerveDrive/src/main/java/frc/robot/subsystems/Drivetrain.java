package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.Constants.DriveConstants;
import frc.robot.utils.RobotMap;

public class Drivetrain extends SubsystemBase{
    private static Drivetrain drivetrain;
    private final SwerveModule[] swerveModule;
    private final SwerveModule frontLefModule, frontRightModule, backLeftModule, backRightModule;
    private  SwerveModuleState[] swerveModuleState;
    private final SwerveModulePosition[] swerveModulePosition;
    private final Pigeon2 gyro;
    private double heading;

    public Drivetrain(){
        frontLefModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_LEFT_MODULE_DRIVE_ID, RobotMap.FRONT_LEFT_MODULE_TURN_ID, RobotMap.FRONT_LEFT_MODULE_CANCODER_ID, 0);
        frontRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID, RobotMap.FRONT_RIGHT_MODULE_TURN_ID, RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID, 0);
        backLeftModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_LEFT_MODULE_DRIVE_ID, RobotMap.BACK_LEFT_MODULE_TURN_ID, RobotMap.BACK_LEFT_MODULE_CANCODER_ID, 0);
        backRightModule = new SwerveModule(RobotMap.CANIVORE_NAME, RobotMap.BACK_RIGHT_MODULE_DRIVE_ID, RobotMap.BACK_RIGHT_MODULE_TURN_ID, RobotMap.BACK_RIGHT_MODULE_CANCODER_ID, 0);

        swerveModule = new SwerveModule[]{frontLefModule, frontRightModule, backLeftModule, backRightModule};
        swerveModulePosition = new SwerveModulePosition[]{frontLefModule.getPosition(), frontRightModule.getPosition(), backLeftModule.getPosition(), backRightModule.getPosition()};
        swerveModuleState = Constants.DriveConstants.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

        gyro = new Pigeon2(RobotMap.GYRO_ID, RobotMap.CANIVORE_NAME);
        gyro.setYaw(0);
    }

    public static Drivetrain getInstance(){
        if(drivetrain == null){
            drivetrain = new Drivetrain();
        }
        return drivetrain;
    }

    public void updateModulePositions(){
        for(int i = 0; i < swerveModulePosition.length; i++){
            swerveModulePosition[i] = swerveModule[i].getPosition();
        }
    }

    public Rotation2d getHeadingRotation2d(){
        return gyro.getRotation2d();
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerRotation){
        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        ChassisSpeeds robotRelativeSpeeds;
        if(fieldOriented){
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeadingRotation2d());
        }else{
            robotRelativeSpeeds = fieldRelativeSpeeds;
        }

        swerveModuleState = DriveConstants.kinematics.toSwerveModuleStates(robotRelativeSpeeds);
        optimizeModuleStates();
        setModuleStates(swerveModuleState);
    }

    public void optimizeModuleStates(){
        for(int i = 0; i > swerveModuleState.length; i++){
            swerveModuleState[i] = SwerveModuleState.optimize(swerveModuleState[i], new Rotation2d(swerveModule[i].getCANcoderReading()));
        }
    }

    public void setModuleStates(SwerveModuleState[] statesList){
        for(int i = 0; i < statesList.length; i++){
            swerveModule[i].setDesiredState(statesList[i]);
        }
    }

    public double getHeading(){
        heading = gyro.getAngle();
        return Math.IEEEremainder(heading, 360);
    }
}
