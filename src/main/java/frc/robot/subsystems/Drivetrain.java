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
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.CameraConstants;
import frc.robot.utils.Constants.DriveConstants;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.ModuleConstants;
import frc.robot.utils.RobotMap;
import frc.robot.utils.ShotMap;
import frc.robot.utils.ShotMap.ShotMapValue;

public class Drivetrain extends SubsystemBase {
    private static Drivetrain drivetrain;

    private final SwerveModule frontLeftModule, frontRightModule, backLeftModule, backRightModule;
    private final SwerveModule[] swerveModules;

    private SwerveModuleState[] swerveModuleStates;
    private double[] swerveModulePositionsRadians;
    private final SwerveModulePosition[] swerveModulePositions;

    private final Pigeon2 gyro;
    private final Limelight[] limelights;

    private double heading;
    private double rotation = 0;
    private boolean usingMegaTag;
    private SwerveDrivePoseEstimator odometry;
    private static Translation2d currentTranslationBlue;

    private final Field2d fusedOdometry;

    private Drivetrain() {
        // CANBus defaultCANBus = new CANBus(RobotMap.CANIVORE_NAME);
        CANBus defaultCANBus = new CANBus("rio");

        usingMegaTag = SmartDashboard.putBoolean("using mega tag", true);

        limelights = new Limelight[] {
            LimelightFront.getInstance(),
            LimelightLeft.getInstance(),
            LimelightRight.getInstance(),
            LimelightBack.getInstance()
        };

        frontLeftModule = new SwerveModule(
            defaultCANBus,
            RobotMap.FRONT_LEFT_MODULE_DRIVE_ID,
            RobotMap.FRONT_LEFT_MODULE_TURN_ID,
            RobotMap.FRONT_LEFT_MODULE_CANCODER_ID,
            ModuleConstants.kFrontLeftMagnetOffset
        );
        frontRightModule = new SwerveModule(
            defaultCANBus,
            RobotMap.FRONT_RIGHT_MODULE_DRIVE_ID,
            RobotMap.FRONT_RIGHT_MODULE_TURN_ID,
            RobotMap.FRONT_RIGHT_MODULE_CANCODER_ID,
            ModuleConstants.kFrontRightMagnetOffset
        );
        backLeftModule = new SwerveModule(
            defaultCANBus,
            RobotMap.BACK_LEFT_MODULE_DRIVE_ID,
            RobotMap.BACK_LEFT_MODULE_TURN_ID,
            RobotMap.BACK_LEFT_MODULE_CANCODER_ID,
            ModuleConstants.kBackLeftMagnetOffset
        );
        backRightModule = new SwerveModule(
            defaultCANBus,
            RobotMap.BACK_RIGHT_MODULE_DRIVE_ID,
            RobotMap.BACK_RIGHT_MODULE_TURN_ID,
            RobotMap.BACK_RIGHT_MODULE_CANCODER_ID,
            ModuleConstants.kBackRightMagnetOffset
        );

        swerveModules = new SwerveModule[] { frontLeftModule, frontRightModule, backLeftModule, backRightModule };
        swerveModulePositions = new SwerveModulePosition[] {
            frontLeftModule.getPosition(),
            frontRightModule.getPosition(),
            backLeftModule.getPosition(),
            backRightModule.getPosition()
        };
        swerveModulePositionsRadians = new double[] {
            frontLeftModule.getPositionRadians(), frontRightModule.getPositionRadians(),
            backLeftModule.getPositionRadians(), backRightModule.getPositionRadians()
        };
        swerveModuleStates = Constants.DriveConstants.kKinematics.toSwerveModuleStates(
            new ChassisSpeeds(0, 0, 0)
        );

        gyro = new Pigeon2(RobotMap.GYRO_ID, defaultCANBus);
        setGyro(0);
        
        odometry = new SwerveDrivePoseEstimator(
            DriveConstants.kKinematics, getHeadingBlueRotation2d(),
            swerveModulePositions, new Pose2d(),
            VecBuilder.fill(0.2, 0.2, 0.1),
            VecBuilder.fill(0.8, 0.8, 99999999)
        );
        odometry.resetPose(new Pose2d(0, 0, getHeadingBlueRotation2d()));

        fusedOdometry = new Field2d();
        SmartDashboard.putData("Fused Odometry", fusedOdometry);
        
        initializeLogEntries();
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
        for (int i = 0; i < swerveModulePositions.length; i++)
            swerveModulePositions[i] = swerveModules[i].getPosition();
    }

    public Translation2d getCurrentTranslationBlue() {
        return currentTranslationBlue;
    }

    public void drive(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerRotation) {
        this.rotation = rotation;
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            currentTranslationBlue = translation;
        else
            currentTranslationBlue = translation.rotateBy(Rotation2d.fromDegrees(180));

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        ChassisSpeeds robotRelativeSpeeds;

        if (fieldOriented)
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, Rotation2d.fromDegrees(getHeading()));
        else
            robotRelativeSpeeds = fieldRelativeSpeeds;

        if (centerRotation == null)
            swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        else
            swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(robotRelativeSpeeds, centerRotation);

        for (int i = 0; i > swerveModuleStates.length; i++)
            swerveModuleStates[i].optimize(new Rotation2d(swerveModules[i].getCANcoderReading()));

        setModuleStates(swerveModuleStates);
    }


    public void driveBlue(Translation2d translation, double rotation, boolean fieldOriented, Translation2d centerRotation) {
        this.rotation = rotation;
        currentTranslationBlue = translation;

        ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds(translation.getX(), translation.getY(), rotation);
        ChassisSpeeds robotRelativeSpeeds;

        if (fieldOriented)
            robotRelativeSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(fieldRelativeSpeeds, getHeadingBlueRotation2d());
        else
            robotRelativeSpeeds = fieldRelativeSpeeds;

        if (centerRotation == null)
            swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        else
            swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(robotRelativeSpeeds, centerRotation);

        for (int i = 0; i > swerveModuleStates.length; i++)
            swerveModuleStates[i].optimize(new Rotation2d(swerveModules[i].getCANcoderReading()));

        setModuleStates(swerveModuleStates);
    }


        /**
     * optimizes the angle in each module state, will turn the closer direction
     */
    public void optimizeModuleStates() {
        for (int i = 0; i < swerveModuleStates.length; i++) {
            swerveModuleStates[i].optimize(new Rotation2d(swerveModules[i].getCANcoderReading()));
        }
    }

    double currentHeadingDirection = 0;

    public double getRotationOverride() {
        return 5 * currentHeadingDirection;
    }

       /**
     * Only used during autonomous, sets driving strictly to robot relative
     * 
     * @param robotRelativeSpeeds
     */
    public void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
        // System.err.println("DRIVE ROBOT RELATIVE time " + Timer.getFPGATimestamp() + " " + robotRelativeSpeeds.vxMetersPerSecond + " " + robotRelativeSpeeds.vyMetersPerSecond + " rot " + robotRelativeSpeeds.omegaRadiansPerSecond);
        // SmartDashboard.putNumber(robotRelativeSpeeds.vxMetersPerSecond

        currentHeadingDirection = Math.atan2(robotRelativeSpeeds.vyMetersPerSecond, robotRelativeSpeeds.vxMetersPerSecond);

        ChassisSpeeds fieldRelativeSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(robotRelativeSpeeds, getHeadingBlueRotation2d());
        currentTranslationBlue = new Translation2d(fieldRelativeSpeeds.vxMetersPerSecond, fieldRelativeSpeeds.vyMetersPerSecond);

        swerveModuleStates = DriveConstants.kKinematics.toSwerveModuleStates(robotRelativeSpeeds);
        setModuleStates(swerveModuleStates);
    }

    /**
     * @return returns current robot relative chassis speeds by getting the state of each module and 
     * then converting to chassis speeds
     */
    public ChassisSpeeds getRobotRelativeSpeeds() {
        return DriveConstants.kKinematics.toChassisSpeeds(
            frontLeftModule.getState(),
            frontRightModule.getState(),
            backLeftModule.getState(),
            backRightModule.getState()
        );
    }

    public double getYaw() {
        return gyro.getYaw().getValueAsDouble();
    }

    public double[] getSwerveModulePositionsRadians() {
        return swerveModulePositionsRadians;
    }

    public void setModuleStates(SwerveModuleState[] statesList) {
        for (int i = 0; i < statesList.length; i++) {
            swerveModules[i].setOptimizedState(statesList[i]);
            swerveModulePositionsRadians[i] = swerveModules[i].getPositionRadians();
        }
    }

    public double getHeading() {
        heading = gyro.getRotation2d().getDegrees();
        return Math.IEEEremainder(heading, 360);
    }

    public double getHeadingBlue() {
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue)
            return getHeading();
        return Math.IEEEremainder(gyro.getYaw().getValueAsDouble() + 180, 360);
    }

    public Rotation2d getHeadingRotation2d() {
        return gyro.getRotation2d();
    }

    public Rotation2d getHeadingBlueRotation2d() {
        return Rotation2d.fromDegrees(getHeadingBlue());
    }

    public void lockModules() {
        frontLeftModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(Math.PI / 4)));
        backLeftModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(3 * Math.PI / 4)));
        frontRightModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(-Math.PI / 4)));
        backRightModule.setOptimizedState(new SwerveModuleState(0, new Rotation2d(-3 * Math.PI / 4)));
    }

    public void updateOdometry(){
        odometry.update(getHeadingBlueRotation2d(), swerveModulePositions);
        if (!DriverStation.isAutonomous() && usingMegaTag) {
            for (Limelight ll : limelights)
                ll.fuseEstimatedPose(odometry);
        }
    }
    public Pose2d getPose() {
        return new Pose2d(
            odometry.getEstimatedPosition().getTranslation(),
            getHeadingBlueRotation2d()
        );
    }

    public void setPose(Pose2d pose){
        System.out.println("PATHPLANNER SET POSE!!!");
        gyro.reset();
        odometry.resetPosition(getHeadingBlueRotation2d(), swerveModulePositions,pose);
    }

    public void resetTranslation(Translation2d translation) {
        odometry.resetTranslation(translation);
    }

    public void setStartingPose(Pose2d pose) {
        gyro.setYaw(pose.getRotation().getDegrees());

        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
            pose = new Pose2d(FieldConstants.kFieldSize.minus(pose.getTranslation()), pose.getRotation());

        odometry.resetPosition(pose.getRotation(), swerveModulePositions,pose);
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
        SwerveModuleState[] moduleStates = Arrays.copyOf(swerveModuleStates, 4);

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
        SmartDashboard.putNumber("Heading", getHeading());
        SmartDashboard.putNumber("Blue Heading", getHeadingBlue());
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

    private DoubleLogEntry gyroAngleEntry, gyroAngleEntryBlue;
    private DoubleLogEntry xAccelEntry, yAccelEntry, zAccelEntry, desiredSpeedEntry;

    private DoubleArrayLogEntry moduleSpeedsDesiredEntry, modulePositionsDesiredEntry;
    private DoubleArrayLogEntry moduleSpeedsActualEntry, modulePositionsActualEntry;
    
    private void initializeLogEntries() {
        DataLog log = DataLogManager.getLog();

        gyroAngleEntry = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle");
        gyroAngleEntryBlue = new DoubleLogEntry(log, "/Drivetrain/Gyro Angle Blue");

        xAccelEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain X Accel");
        yAccelEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Y Accel");
        zAccelEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Z Accel");

        desiredSpeedEntry = new DoubleLogEntry(log, "/Drivetrain/Drivetrain Desired Speed");
        moduleSpeedsDesiredEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Desired Speeds");
        modulePositionsDesiredEntry = new DoubleArrayLogEntry(log, "/Drivetrain/Swerve Module Desired Positions");
    }

    public void updateDrivetrainLogs() {
        gyroAngleEntry.append(getHeading());
        gyroAngleEntryBlue.append(getHeadingBlue());

        xAccelEntry.append(gyro.getAccelerationX().getValueAsDouble());
        yAccelEntry.append(gyro.getAccelerationY().getValueAsDouble());
        zAccelEntry.append(gyro.getAccelerationZ().getValueAsDouble());
        desiredSpeedEntry.append(currentTranslationBlue.getNorm());

        double[] swerveModulePositions = {
            swerveModuleStates[0].angle.getDegrees(),
            swerveModuleStates[1].angle.getDegrees(),
            swerveModuleStates[2].angle.getDegrees(),
            swerveModuleStates[3].angle.getDegrees()
        };
        double[] swerveModuleSpeeds = {
            swerveModuleStates[0].speedMetersPerSecond,
            swerveModuleStates[1].speedMetersPerSecond,
            swerveModuleStates[2].speedMetersPerSecond,
            swerveModuleStates[3].speedMetersPerSecond
        };
                
        double[] swerveModulePositionsActual = {
            swerveModules[0].getAngle(),
            swerveModules[1].getAngle(),
            swerveModules[2].getAngle(),
            swerveModules[3].getAngle()
        };
        double[] swerveModuleSpeedsActual = {
            swerveModules[0].getVelocity(),
            swerveModules[1].getVelocity(),
            swerveModules[2].getVelocity(),
            swerveModules[3].getVelocity()
        };

        moduleSpeedsDesiredEntry.append(swerveModuleSpeeds);
        modulePositionsDesiredEntry.append(swerveModulePositions);
        moduleSpeedsActualEntry.append(swerveModuleSpeedsActual);
        modulePositionsActualEntry.append(swerveModulePositionsActual);
    }
}


