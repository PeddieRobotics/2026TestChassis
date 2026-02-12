package frc.robot.subsystems;

import java.util.Arrays;
import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.CameraConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.RobotMap;
import frc.robot.utils.ShotMap;
import frc.robot.utils.ShotMap.ShotMapValue;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain drivetrain;
    private final SwerveModule[] swerveModule;
    private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    private SwerveModuleState[] swerveModuleState;
    private double[] swerveModulePositionsRadians;
    private final SwerveModulePosition[] swerveModulePosition;
    private final Pigeon2 gyro;
    private final Limelight[] limelights = new Limelight[CameraConstants.kNumberLimelights];
    // private final LimelightFront frontLimelight; 
    // private final LimelightBack backLimelight; 
    // private final LimelightLeft leftLimelight;
    // private final LimelightRight rightLimelight;
    
    private double heading;
    private double rotation = 0;
    private boolean usingMegaTag;
    private SwerveDrivePoseEstimator odometry;
    private static Translation2d currentTranslation;
    private final Field2d fusedOdometry;

    private Drivetrain() {
        // CANBus defaultCANBus = new CANBus(RobotMap.CANIVORE_NAME);
        CANBus defaultCANBus = new CANBus("rio");

        usingMegaTag = SmartDashboard.putBoolean("using mega tag", true);

        limelights[0] = LimelightFront.getInstance();
        limelights[1] = LimelightBack.getInstance();
        limelights[2] = LimelightLeft.getInstance();
        limelights[3] = LimelightRight.getInstance();

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
        swerveModulePositionsRadians = new double[] {
            frontLeftModule.getPositionRadians(), frontRightModule.getPositionRadians(),
            backLeftModule.getPositionRadians(), backRightModule.getPositionRadians()
        };
        swerveModuleState = Constants.DriveConstants.kinematics.toSwerveModuleStates(
            new ChassisSpeeds(0, 0, 0)
        );

        gyro = new Pigeon2(RobotMap.GYRO_ID, defaultCANBus);
        setGyro(0);
        
        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kKinematics, Rotation2d.fromDegrees(getHeadingBlue()), 
            swerveModulePosition, new Pose2d(),
            VecBuilder.fill(0.2, 0.2, 0.1),
            VecBuilder.fill(0.8, 0.8, 99999999)
        );
        odometry.resetPose(new Pose2d(0, 0, Rotation2d.fromDegrees(getHeadingBlue())));

        fusedOdometry = new Field2d();
        SmartDashboard.putData("Fused Odometry", fusedOdometry);
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

    public Translation2d getCurrentTranslation() {
        return currentTranslation;
    }


    public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerRotation) {
        this.rotation = rotation;
        currentTranslation = translation;

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

    public double getYaw() {
        return gyro.getYaw().getValueAsDouble();
    }

    public double[] getSwerveModulePositionsRadians() {
        return swerveModulePositionsRadians;
    }

    public void setModuleStates(SwerveModuleState[] statesList) {
        for (int i = 0; i < statesList.length; i++) {
            swerveModule[i].setOptimizedState(statesList[i]);
            swerveModulePositionsRadians[i] = swerveModule[i].getPositionRadians();
        }
    }

    public double getHeading() {
        heading = gyro.getRotation2d().getDegrees();
        return Math.IEEEremainder(heading, 360);
    }

    public double getHeadingBlue(){
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            return getHeading();
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble() + 180, 360);
    }

    public void lockModules() {
        frontLeftModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4)));
        backLeftModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)));
        frontRightModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)));
        backRightModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(-3 * Math.PI / 4)));
    }

    public void updateOdometry(){
        odometry.update(Rotation2d.fromDegrees(getHeadingBlue()), swerveModulePosition);
        if (!DriverStation.isAutonomous() && usingMegaTag) {
            for (Limelight ll : limelights)
                ll.fuseEstimatedPose(odometry);
        }
    }
    public Pose2d getPose(){
        return odometry.getEstimatedPosition();
    }

    public void setPose(Pose2d pose){
        gyro.reset();
        odometry.resetPosition(Rotation2d.fromDegrees(getHeadingBlue()), swerveModulePosition,pose);
    }

    public void resetTranslation(Translation2d translation) {
        odometry.resetTranslation(translation);
    }

    public void setStartingPose(Pose2d pose) {
        gyro.setYaw(pose.getRotation().getDegrees());

        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            pose = new Pose2d(16.54 - pose.getX(),8.07 - pose.getY(), pose.getRotation());

        odometry.resetPosition(pose.getRotation(), swerveModulePosition,pose);
    }
    
    public double getYawRate() {
        return rotation;
    }

    /**
     * @return returns rotational velocity in DPS
     */
    public double getRotationalVelocity(){
        return -gyro.getAngularVelocityZWorld().getValueAsDouble();
    }

    public double getDrivetrainCurrentVelocity(){
        //gets rotational velocity of the whole robot
        double currentRotationalVelocity = -getRotationalVelocity()*2*Math.PI/360;


        if (Math.abs(currentRotationalVelocity)<0.05){
            currentRotationalVelocity = 0;
        }

        //gets rotational velocity of each module
        ChassisSpeeds rotationalVelocity = new ChassisSpeeds(0,0, currentRotationalVelocity);
        SwerveModuleState pureRotationalStates[] = DriveConstants.kKinematics.toSwerveModuleStates(rotationalVelocity);
     
        //gets rotational and translational velocity of each module
        SwerveModuleState[] moduleStates = Arrays.copyOf(swerveModuleState, 4);

        Translation2d[] fullModuleStates = new Translation2d[4];
        Translation2d[] pureTranslationalStates = new Translation2d[4];


    for (int i = 0; i<4; i++){
            fullModuleStates[i] = new Translation2d(moduleStates[i].speedMetersPerSecond, moduleStates[i].angle);

            Translation2d pureRotation = new Translation2d(pureRotationalStates[i].speedMetersPerSecond, pureRotationalStates[i].angle);

            //subtracts rotational velocity from full states, leaving only translational velocity
            pureTranslationalStates[i] = fullModuleStates[i].minus(pureRotation);
        }

        double averageModuleSpeed = 0;

        for(Translation2d moduleSpeed : pureTranslationalStates){
            double moduleSpeedMagnitude = moduleSpeed.getNorm();

            averageModuleSpeed += moduleSpeedMagnitude / 4.0;
        }

        return averageModuleSpeed;
    }


    @Override
    public void periodic() {
        updateModulePositions();
        updateOdometry();

        fusedOdometry.setRobotPose(odometry.getEstimatedPosition());
        
        SmartDashboard.putNumber("Yaw rate", getYawRate());
        usingMegaTag = SmartDashboard.getBoolean("using mega tag", false);
        if (SmartDashboard.getBoolean("Test Map?", false)) {
            double distance = SmartDashboard.getNumber("Test Distance", 0);
            double v_r = SmartDashboard.getNumber("Test Rad. Vel.", 0);
            ShotMapValue value = ShotMap.queryShotMap(distance, v_r);

            SmartDashboard.putNumber("Test Output: Speed", value.exit_v());
            SmartDashboard.putNumber("Test Output: Pitch", value.pitch());
        }

        SmartDashboard.putNumber("Odometry x", odometry.getEstimatedPosition().getX());
        SmartDashboard.putNumber("Odometry y", odometry.getEstimatedPosition().getY());
    }
}