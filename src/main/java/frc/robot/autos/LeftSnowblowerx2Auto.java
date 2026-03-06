package frc.robot.autos;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.TrenchAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.FieldConstants.TrenchLocations;
import frc.robot.utils.Constants.TrenchAlignConstants;

public class LeftSnowblowerx2Auto {
    public static final Command auto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            Drivetrain.getInstance().setStartingPose(new Pose2d(4.466, 7.367, Rotation2d.fromDegrees(0)));
            Autonomous.startPassDrive();
        }),
        new AutoDriveCommand(
            List.of(
                new Pose2d(4.466, 7.367, Rotation2d.fromDegrees(0)),
                new Pose2d(6.278, 7.367, Rotation2d.fromDegrees(0))
            ),
            new PathConstraints(1.5, 3, 5 * Math.PI, 6 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(0.75, Rotation2d.fromDegrees(-90))
        ),
        new ParallelCommandGroup(
        new InstantCommand(() -> {
            Autonomous.startSnakeDrive();
        }
        ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(6.278, 7.367, Rotation2d.fromDegrees(0)),
                new Pose2d(6.778, 6.385, Rotation2d.fromDegrees(-66.501)),
                new Pose2d(6.778, 1.104, Rotation2d.fromDegrees(130.624)),
                new Pose2d(6.89, 6.810, Rotation2d.fromDegrees(-62.990)),
                new Pose2d(6.89, 1.400, Rotation2d.fromDegrees(-96.233))
            ),
            List.of(
                new RotationTarget(1.25, Rotation2d.fromDegrees(-90)),
                new RotationTarget(1.75, Rotation2d.fromDegrees(-90)),
                new RotationTarget(2.25, Rotation2d.fromDegrees(90)),
                new RotationTarget(2.75, Rotation2d.fromDegrees(90)),
                new RotationTarget(3.25, Rotation2d.fromDegrees(-90)),
                new RotationTarget(3.75, Rotation2d.fromDegrees(-90))
            ),
            List.of(
            ),
            new PathConstraints(0.75, 3, 5 * Math.PI, 6 * Math.PI),
            new IdealStartingState(0.75, Rotation2d.fromDegrees(-90)),
            new GoalEndState(0, Rotation2d.fromDegrees(-91))
            //new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(-91))
        ))
        //new TrenchAlign(true)
    );
}
