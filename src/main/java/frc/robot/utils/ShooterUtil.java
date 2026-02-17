package frc.robot.utils;

import java.io.IOException;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.TurretConstants;
import frc.robot.utils.Constants.FieldConstants.PassingLocations;
import frc.robot.utils.ShotMap.ShotMapValue;

public class ShooterUtil {
    public static record ShootingParameters(double pitch, Rotation2d yaw, double rpm) {};

    private static Field2d virtualTarget;

    public static void initShooterUtils(String filename) throws IOException {
        virtualTarget = new Field2d();
        SmartDashboard.putData("Virtual target", virtualTarget);
        ShotMap.initShotMap(filename);
    }

    
    public static ShootingParameters getShootingParameters(
        Translation2d turretPose,
        Translation2d robotVelocity,
        Translation2d turretToHub
    ) {
        final double turretToHubAngle = turretToHub.getAngle().getRadians();

        final Translation2d R = new Translation2d( Math.cos(turretToHubAngle), Math.sin(turretToHubAngle));
        final Translation2d T = new Translation2d(-Math.sin(turretToHubAngle), Math.cos(turretToHubAngle));

        final double v_r = R.dot(robotVelocity);
        final double v_t = T.dot(robotVelocity);

        // iteration 0: actual hub
        final Translation2d hub0 = FieldConstants.getHub();
        final Translation2d turretToHub0 = turretToHub;

        // iteration 1
        final ShotMapValue mapVal0 = ShotMap.queryShotMap(turretToHub0.getNorm(), v_r);
        final Translation2d distanceInAir0 = T.times(v_t * mapVal0.flightTime());
        final Translation2d hub1 = hub0.minus(distanceInAir0);
        final Translation2d turretToHub1 = hub1.minus(turretPose);

        // iteration 2, remove if not necessary (but not mapVal1)
        final ShotMapValue mapVal1 = ShotMap.queryShotMap(turretToHub1.getNorm(), v_r);
        final Translation2d distanceInAir1 = T.times(v_t * mapVal1.flightTime());
        final Translation2d hub2 = hub1.minus(distanceInAir1);
        final Translation2d turretToHub2 = hub2.minus(turretPose);
        
        // put values on elastic for visual 
        SmartDashboard.putNumber("robot velocity", robotVelocity.getNorm());
        SmartDashboard.putNumber("robot velocity x", robotVelocity.getX());
        SmartDashboard.putNumber("robot velocity y", robotVelocity.getY());
        SmartDashboard.putNumber("robot radial velocity", v_r);
        SmartDashboard.putNumber("robot tangential velocity", v_t);
        
        virtualTarget.setRobotPose(new Pose2d(hub2, new Rotation2d()));

        return new ShootingParameters(
            mapVal1.pitch(),
            turretToHub2.getAngle(),
            getRPM(mapVal1.exit_v())
        );
    }
    
    private static double getRPM(double exit_v) {
        // TODO 
        return exit_v * 300;
    }

    // sets passing location accrosing to robot pose and alliance
    public static Translation2d getPassingLocation() {
        final Translation2d depotLocation, outpostLocation;
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == DriverStation.Alliance.Blue) {
            depotLocation = PassingLocations.kBlueDepot;
            outpostLocation = PassingLocations.kBlueOutpost;
        }
        else {
            depotLocation = PassingLocations.kRedDepot;
            outpostLocation = PassingLocations.kRedOutpost;
        }
        
        Translation2d robotCenter = Drivetrain.getInstance().getPose().getTranslation();
        Translation2d turretCenter = robotCenter.plus(TurretConstants.kRobotCenterToTurretCenter.rotateBy(Rotation2d.fromDegrees(Drivetrain.getInstance().getHeadingBlue())));

        double turretY = turretCenter.getY();

        final Translation2d bestLocation;
        if (Math.abs(depotLocation.getY() - turretY) < Math.abs(outpostLocation.getY() - turretY))
            bestLocation = depotLocation;
        else
            bestLocation = outpostLocation;

        return bestLocation;
    }
}
