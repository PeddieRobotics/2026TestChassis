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
        // places rotError between -180 and 180, then normalize between -1 and 1
        double rotError = Math.IEEEremainder(target - gyro, 360) / 180;

        // for small errors, do nothing
        if (Math.abs(rotError) < 0.05)
            return 0;

        // 4 is a PID factor
        double rotVelocity = rotError * 4;

        // cap velocity at 1
        if (Math.abs(rotVelocity) > 1)
            return 1.0 * Math.signum(rotVelocity);

        return rotVelocity;
    }

    @Override
    public void initialize() {
        oi = OI.getInstance();
    }

    @Override
    public void execute() {

        Translation2d translation = oi.getSwerveTranslation();
        double rotation = oi.getRotation();
        
        drivetrain.drive(translation, rotation, true, null);

        // if (DriverStation.isAutonomous())
        //     return;
            
        // Translation2d translation = oi.getSwerveTranslation();
        // double rotation;
        
        // // bumpers are not being pressed, snake is onned
        // if (oi.getRotation() == 0 && oi.getJoystickAngle() != 0)
        //     rotation = getRotationControl(oi.getJoystickAngle(), drivetrain.getHeading());
        // else
        //     rotation = oi.getRotation();

        // if (oi.getDriverDPadInput() != DPadDirection.NONE)
        //     translation = oi.getCardinalDirection();

        // drivetrain.drive(translation, rotation, true, null);
    }

    @Override
    public void end(boolean interrupted) {}

    @Override
    public boolean isFinished() {
        return false;
    }
}
