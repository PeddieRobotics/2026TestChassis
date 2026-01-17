// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.SwerveModule;
import frc.robot.utils.OI;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class SwerveDriveCommand extends Command {
  private Drivetrain drivetrain;
  private OI oi;

  public SwerveDriveCommand() {
    drivetrain = Drivetrain.getInstance();
    oi = OI.getInstance();

    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation2d translation = oi.getSwerveTranslation();
    double rotation = oi.getRotation();

    drivetrain.drive(new Translation2d(0, 0), rotation, true, new Translation2d(0, 0));
    // drivetrain.drive(translation, rotation, isScheduled(), translation);

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
