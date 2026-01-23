// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.OI;
import frc.robot.utils.OI.DPadDirection;;



public class SwerveDriveCommand extends Command {

    private Drivetrain drivetrain;
    private OI oi;

    public SwerveDriveCommand() {
        drivetrain = Drivetrain.getInstance();

        addRequirements(drivetrain);
    }

    public double getRotationControl(double target, double gyro){
        double delta = target-gyro;
        double rotSpeed;
        if (delta > 180) {
            rotSpeed = (delta-360)/180;
        }
        else if (Math.abs(delta) <= 180) {
            rotSpeed = delta/180;
        }
        else {
            rotSpeed = (delta+360)/180;
        }

        if (Math.abs(rotSpeed) < 0.05){
            return 0;
        }

        rotSpeed *= 4;
        if (Math.abs(rotSpeed) > 1) {
            return 1.0 * Math.signum(rotSpeed);
        }
        return rotSpeed;
    }

    @Override
    public void initialize() {
        oi = OI.getInstance();
    }

    @Override
    public void execute() {
        SmartDashboard.putNumber("Joystick Angle", oi.getJoystickAngle());
        SmartDashboard.putNumber("Heading", drivetrain.getHeading());
        
        if (DriverStation.isAutonomous())
            return;
            
        Translation2d translation = oi.getSwerveTranslation();
        double rotation;
        
        if (oi.getRotation() == 0 && oi.getJoystickAngle() != 0){ //bumpers are not being pressed, snake is onned
            rotation = getRotationControl(oi.getJoystickAngle(), drivetrain.getHeading());
            SmartDashboard.putNumber("Rotation", rotation);
        }
        else {
            rotation = oi.getRotation();
        }
        boolean isFieldOriented = true;

        if (oi.getDriverDPadInput() != DPadDirection.NONE) {
            translation = oi.getCardinalDirection();
            // if (Superstructure.getInstance().isClimbState()) {
            //     translation = translation.times(-1);
            //     isFieldOriented = false;
            // }
        }

        drivetrain.drive(translation, rotation, isFieldOriented, new Translation2d(0, 0));
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
