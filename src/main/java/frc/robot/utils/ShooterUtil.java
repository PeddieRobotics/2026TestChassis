package frc.robot.utils;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.TurretConstants;

public class ShooterUtil {
    public static class ShootingParameters {
        public static double pitch, pitchMin, pitchMax;
        public static double yaw;
        public static double shotVelocity;

        public ShootingParameters() {
        } // constructor
    }

    // in robot-relative coords
    public static ShootingParameters getShootingParameters(Translation2d robot_velocity,
            Translation2d turretDistanceToHub, double timeInAir) {
        ShootingParameters allParams = new ShootingParameters(); // to set all values
        // find turret to hub angle (in between), convert turretangle using constant
        // deviations
        double turretToHubAngle = Math.atan2(turretDistanceToHub.getY() + TurretConstants.kTurretDisplacementY,
                turretDistanceToHub.getX() + TurretConstants.kTurretDisplacementY);
        // use projection vector of robot_velocity (vector) to find rad_vec, tan_vec
        Translation2d radial_vector = new Translation2d(Math.cos(turretToHubAngle), Math.sin(turretToHubAngle));
        Translation2d tangent_vector = new Translation2d(-1 * Math.sin(turretToHubAngle), Math.cos(turretToHubAngle));
        double radial_velocity = robot_velocity.dot(radial_vector);
        double tangent_velocity = robot_velocity.dot(tangent_vector);
        // calculate yaw using given theta (calculated before func based on rotation or
        // not) + extra stuff (calculated here)
        // use time in air because it takes time for the turret to turn and we're
        // accounting for extra distance covered
        allParams.yaw = turretToHubAngle + Math.atan2(timeInAir * tangent_velocity, turretDistanceToHub.getNorm());
        // run polynomial on radial_vel, distance_to_hub and get i) pitch and ii) shot
        // velo (in 3d now that all 2d stuff is done)
        allParams.pitch = evaluateShotPitch(radial_velocity, turretDistanceToHub.getNorm());
        allParams.shotVelocity = evaluateShotVelocity(radial_velocity, turretDistanceToHub.getNorm());
        // use func ig
        // allParams.pitchMin = Math.toRadians(35); --> use model to figure this out
        // allParams.pitchMax = Math.toRadians(85); --> use model to figure this out
        return allParams;
    }

    public static double evaluateShotVelocity(double radial_vel, double distance) {
        double shooterSpeed = 0;
        // shooterSpeed = polynomial;
        // use 1st polynomial constants to eval
        return 0.0;
    }

    public static double evaluateShotPitch(double radial_vel, double distance) {
        // use 2nd polynomial constants to eval
        double shooterPitch = 0;
        // shooterPitch =polynomial;
        return 0.0; 
    }

    // sets passing location accrosing to robot pose and alliance
    public static Translation2d getPassingLocation(Translation2d robotPose) {
        boolean topSide = (int)robotPose.getY() / 4 != 0;
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == Alliance.Blue) {
            if (topSide) {
                return (new Translation2d(FieldConstants.topLeftCornerX, FieldConstants.topLeftCornerY)).minus(robotPose);
            } 
            else {
                return (new Translation2d(FieldConstants.bottomLeftCornerX, FieldConstants.bottomLeftCornerY)).minus(robotPose);
            }
        } 
        else {
            if (topSide) {
                return (new Translation2d(FieldConstants.topRightCornerX, FieldConstants.topRightCornerY)).minus(robotPose);
            } 
            else {
                return (new Translation2d(FieldConstants.bottomRightCornerX, FieldConstants.bottomRightCornerY)).minus(robotPose);
            }
        }
    }
}
