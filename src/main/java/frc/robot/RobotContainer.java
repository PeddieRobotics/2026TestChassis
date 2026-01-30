// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Properties;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.io.InputStream;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.Hopper;
import frc.robot.utils.ShotMap;

public class RobotContainer {
    private Drivetrain drivetrain;
    // private Flywheel flywheel;
    // private Hopper hopper;

    public RobotContainer() throws IOException {
        ShotMap.initShotMap("shotdata.txt");
        SmartDashboard.putBoolean("Test Map?", false);
        SmartDashboard.putNumber("Test Distance", 0);
        SmartDashboard.putNumber("Test Rad. Vel.", 0);

        drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new SwerveDriveCommand());

        // flywheel = Flywheel.getInstance();
        // hopper = Hopper.getInstance();
    }

    public Command getAutonomousCommand() {
        return new InstantCommand();
    }
}
