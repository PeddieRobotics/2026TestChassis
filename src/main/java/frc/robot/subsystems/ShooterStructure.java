// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.lang.reflect.Parameter;
import java.util.Arrays;
import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.deser.impl.CreatorCandidate.Param;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.HoodConstants;
import frc.robot.utils.Constants.ScoreConstants;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.Constants.TurretConstants;
import frc.robot.utils.ShooterUtil.ShootingParameters;
import frc.robot.utils.LiveData;
import frc.robot.utils.Logger;
import frc.robot.utils.Shifts;
import frc.robot.utils.ShooterUtil;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.ParameterConversion;

import static frc.robot.subsystems.Superstructure.SuperstructureState.*;

public class ShooterStructure extends SubsystemBase {
    private static ShooterStructure shooterStructure;
    private ShooterStructureState shooterState;
    private boolean isOperatorOverride;
    private Drivetrain drivetrain;
    private Flywheel shooter;
    private Hopper hopper;
    private Timer timer;
    private ShootingParameters params;
    private Hood hood;
    private Turret turret;

    private StringLogEntry shooterStructureCurrentStateEntry;

    public enum ShooterStructureState {
        HOLD,
        SCORE,
        PASS
    }

    public ShooterStructure() {
        shooterState = ShooterStructureState.HOLD;
        shooter = Flywheel.getInstance();
        hopper = Hopper.getInstance();
        params = new ShootingParameters(); // constructing nulls for now
        hood = Hood.getInstance();
        turret = Turret.getInstance();
        drivetrain = Drivetrain.getInstance();
        isOperatorOverride = false;

        DataLog log = DataLogManager.getLog();

        shooterStructureCurrentStateEntry = new StringLogEntry(log, "/ShooterStructure/Current ShooterStructure State");
    }
    
    public static ShooterStructure getInstance() {
        if (shooterStructure == null)
            shooterStructure = new ShooterStructure();
        return shooterStructure;
    }

    public void setOperatorOverride(boolean isOperatorOverride) {
        this.isOperatorOverride = isOperatorOverride;
    }

    public ShooterStructureState determineStateTeleop() {
        if (Shifts.isHoldThreshold() && isInAllianceZone()) 
            return ShooterStructureState.HOLD;
        if (Shifts.isActive() && isInAllianceZone())
            return ShooterStructureState.SCORE;
        if (isInNeutralZone() || isInOpponentZone())
            return ShooterStructureState.PASS;
        return ShooterStructureState.HOLD;
    }

    private boolean isInAllianceZone() {
        Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
        if (alliance == DriverStation.Alliance.Blue)
            return Drivetrain.getInstance().getPose().getX() <= Units.inchesToMeters(182.11);
        return Drivetrain.getInstance().getPose().getX() >= Units.inchesToMeters(651.22 - 182.11); 
    }

    private boolean isInOpponentZone() {
        Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
        if (alliance == DriverStation.Alliance.Red)
            return Drivetrain.getInstance().getPose().getX() <= Units.inchesToMeters(182.11);
        return Drivetrain.getInstance().getPose().getX() >= Units.inchesToMeters(651.22 - 182.11); 
    }

    private boolean isInNeutralZone() {
        return (Drivetrain.getInstance().getPose().getX() <= Units.inchesToMeters(651.22 - 182.11) && Drivetrain.getInstance().getPose().getX() >= Units.inchesToMeters(182.11));
    }

    public ShooterStructureState getCurrentState(){
        return shooterState;
    }

    public void updateShooterStructureLogs() {
        shooterStructureCurrentStateEntry.append(getCurrentState().toString());
    }

    public void setParams(Translation2d robotVel, Translation2d distToHubTurret, double timeOfRotation) {
        // give it a robot_velocity, turretDistToHub, timeOfRotation(inAir)
        params = ShooterUtil.getShootingParameters(robotVel, distToHubTurret, timeOfRotation);
    }

    public void overrideState(ShooterStructureState wantedState) {
        shooterState = wantedState;
    }



    @Override
    public void periodic() {
        Translation2d robotVector = new Translation2d(drivetrain.getDrivetrainCurrentVelocity(), drivetrain.getHeading());
        double timeOfRot = 0.0;
        switch (shooterState) {
            case HOLD:
                hopper.stopHopper();
                break;

            case SCORE:
                Translation2d robotToHubDist = new Translation2d(ParameterConversion.robotDistanceToHubX(drivetrain.getPose()), 
                ParameterConversion.robotDistanceToHubY(drivetrain.getPose()));
                // set it every single time
                // use created drivetrain object in order to calculate robot_vel, distToHub, timeOfRot
                // use simulation (O(1), constant(s)) to find timeOfRotation (x^3 +x^2y+xy^2+y^3 form), using 2d dist, height diff
                timeOfRot = ShooterConstants.aTimeInAir*Math.pow(robotToHubDist.getNorm(),3) + 
                ShooterConstants.bTimeInAir*Math.pow(robotToHubDist.getNorm(),2)*TurretConstants.kTurretToHubHeight + 
                ShooterConstants.cTimeInAir*robotToHubDist.getNorm()*Math.pow(TurretConstants.kTurretToHubHeight,2) + 
                Math.pow(TurretConstants.kTurretToHubHeight,3);
                setParams(robotVector,robotToHubDist,timeOfRot);
                hood.setHoodAngle(params.pitch);
                shooter.setShooterVelocityLeft(params.shotVelocity); //currently only for the left motor(add right one later)
                turret.setAngle(new Rotation2d(params.yaw));
                hopper.runHopperShoot();
                break;

            case PASS:
                // constant velocity and position to send to
                // depending on red/blue alliance
                Translation2d robotToPassPos = ShooterUtil.getPassingLocation(robotVector);
                hood.setHoodAngle(HoodConstants.kHoodPassingAngle);
                shooter.setShooterVelocityLeft(ShooterConstants.kPassSpeed);
                turret.setAngle(robotToPassPos.getAngle());
                hopper.runHopperShoot();
                break;
        }

        if (!DriverStation.isAutonomous() && !isOperatorOverride)
            shooterState = determineStateTeleop();
    }
}
