// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Turret;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.Constants.TurretConstants;
import frc.robot.utils.ParameterConversion;
import frc.robot.utils.Shifts;
import frc.robot.utils.ShooterUtil;

public class DetermineShooterState extends Command {
    private ShooterStructureState shooterState;
    private boolean isOperatorOverride;
    private Drivetrain drivetrain;
    // private Shooter shooter;
    private Hopper hopper;
    private Timer timer;
    // private Hood hood;
    private Turret turret;

    private StringLogEntry shooterStructureCurrentStateEntry;

    public enum ShooterStructureState {
        HOLD,
        SCORE,
        PASS
    }

    public DetermineShooterState() {
        turret = Turret.getInstance();
        drivetrain = Drivetrain.getInstance();

        addRequirements(turret);
        shooterState = ShooterStructureState.HOLD;
        
        SmartDashboard.putString("Shift", Shifts.determineShift().toString());
        SmartDashboard.putString("ShooterStructure State", shooterState.toString());
    }

    public void setOperatorOverride(boolean isOperatorOverride) {
        this.isOperatorOverride = isOperatorOverride;
    }

    public ShooterStructureState determineStateTeleop() {
        if (SmartDashboard.getBoolean("use ShooterStructure Logic",true)){
            if (Shifts.isHoldThreshold() && isInAllianceZone()) 
                return ShooterStructureState.HOLD;
            if (Shifts.isActive() && isInAllianceZone())
                return ShooterStructureState.SCORE;
            if (isInNeutralZone() || isInOpponentZone())
                return ShooterStructureState.PASS;
            return ShooterStructureState.HOLD;
        }
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

    public void overrideState(ShooterStructureState wantedState) {
        shooterState = wantedState;
    }


    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        turret.lockOnTurret();
        SmartDashboard.putString("ShooterStructure State", shooterState.toString());
        SmartDashboard.putString("Shift", Shifts.determineShift().toString());
        Translation2d robotVector = new Translation2d(drivetrain.getDrivetrainCurrentVelocity(), drivetrain.getHeading());
        double timeOfRot = 0.0;
        switch (shooterState) {
            case HOLD:
                // hopper.stopHopper();
                break;

            case SCORE:
                Translation2d robotToHubDist = new Translation2d(ParameterConversion.robotDistanceToHubX(drivetrain.getPose()), 
                ParameterConversion.robotDistanceToHubY(drivetrain.getPose()));

                var params = ShooterUtil.getShootingParameters(robotVector, robotToHubDist, timeOfRot);
                // hood.setHoodAngle(params.pitch());
                // shooter.setShooterVelocity(params.exitVelocity());
                // turret.setTurretAngle(params.yaw());
                // hopper.runHopperShoot();
                break;

            case PASS:
                // constant velocity and position to send to
                // depending on red/blue alliance
                Translation2d robotToPassPos = ShooterUtil.getPassingLocation(robotVector);
                // hood.setHoodAngle(HoodConstants.kHoodPassingAngle);
                // shooter.setShooterVelocity(ShooterConstants.kPassSpeed);
                // turret.setTurretAngle(robotToPassPos.getAngle().getDegrees());
                // hopper.runHopperShoot();
                break;
        }

        if (!DriverStation.isAutonomous() && !isOperatorOverride)
            shooterState = determineStateTeleop();
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}