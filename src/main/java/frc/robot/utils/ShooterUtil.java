package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.utils.Constants.FieldConstants;

public class ShooterUtil {
    public static record ShootingParameters(double pitch, Rotation2d yaw, double rpm) {};

    private static Field2d shooterUtilOdometry;

    public static void initShooterUtils() {
        shooterUtilOdometry = new Field2d();
        SmartDashboard.putData("shooter util",shooterUtilOdometry);
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

        final var mapVal = ShotMap.queryShotMap(turretToHub.getNorm(), v_r);
        
        final Translation2d distanceInAir = T.times(v_t * mapVal.flightTime());

        final Translation2d effectiveHub = FieldConstants.getHub().minus(distanceInAir);
        
        final Translation2d turretToEffectiveHub = effectiveHub.minus(turretPose);

        final var effectiveMapVal = ShotMap.queryShotMap(turretToEffectiveHub.getNorm(), v_r);
        
        // put values on elastic for visual 
        SmartDashboard.putNumber("robot velocity", robotVelocity.getNorm());
        SmartDashboard.putNumber("robot velocity x", robotVelocity.getX());
        SmartDashboard.putNumber("robot velocity y", robotVelocity.getY());
        SmartDashboard.putNumber("robot radial velocity", v_r);
        SmartDashboard.putNumber("robot tangential velocity", v_t);
        
        shooterUtilOdometry.setRobotPose(new Pose2d(effectiveHub, new Rotation2d()));

        return new ShootingParameters(
            effectiveMapVal.pitch(),
            turretToEffectiveHub.getAngle(),
            getRPM(effectiveMapVal.exit_v())
        );
    }
    
    private static double getRPM(double exit_v) {
        // TODO 
        return exit_v * 300;
    }

    // sets passing location accrosing to robot pose and alliance
    // public static Translation2d getPassingLocation(Translation2d robotPose) {
    //     boolean topSide = (int)robotPose.getY() / 4 != 0;
    //     if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == Alliance.Blue) {
    //         if (topSide) {
    //             return (new Translation2d(FieldConstants.topLeftCornerX, FieldConstants.topLeftCornerY)).minus(robotPose);
    //         } 
    //         else {
    //             return (new Translation2d(FieldConstants.bottomLeftCornerX, FieldConstants.bottomLeftCornerY)).minus(robotPose);
    //         }
    //     } 
    //     else {
    //         if (topSide) {
    //             return (new Translation2d(FieldConstants.topRightCornerX, FieldConstants.topRightCornerY)).minus(robotPose);
    //         } 
    //         else {
    //             return (new Translation2d(FieldConstants.bottomRightCornerX, FieldConstants.bottomRightCornerY)).minus(robotPose);
    //         }
    //     }
    // }
}
