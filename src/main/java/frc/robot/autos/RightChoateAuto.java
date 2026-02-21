package frc.robot.autos;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.TrenchAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.FieldConstants.TrenchLocations;
import frc.robot.utils.Constants.TrenchAlignConstants;

public class RightChoateAuto {
    public static final Command auto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            Drivetrain.getInstance().setStartingPose(new Pose2d(
                4.466, FieldConstants.kFieldSize.getY() - 7.406,
                Rotation2d.fromDegrees(-10.284)
            ));
        }),
        new AutoDriveCommand(
            List.of(
                new Pose2d(
                    4.466, FieldConstants.kFieldSize.getY() - 7.406,
                    Rotation2d.fromDegrees(-10.284)
                ),
                new Pose2d(
                    7.975, FieldConstants.kFieldSize.getY() - 6.264,
                    Rotation2d.fromDegrees(133.132)
                ),
                new Pose2d(
                    6.329, FieldConstants.kFieldSize.getY() - 7.108,
                    Rotation2d.fromDegrees(-148.512)
                )
            ),
            List.of(
                new EventMarker("Stop Snake Drive", 1, new InstantCommand(() -> Autonomous.stopSnakeDrive()))
            ),
            new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
            // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(TrenchAlignConstants.kStage2Speed, Rotation2d.fromDegrees(0))
        ),
        new TrenchAlign(),
        new WaitCommand(1), // !!TODO: change after the wrenchers are ready
        new TrenchAlign(),
        new AutoDriveCommand(
            List.of(
                new Pose2d(
                    TrenchLocations.kRedRightCenter.plus(TrenchAlignConstants.kOutOffset),
                    Rotation2d.fromDegrees(180)
                ),
                new Pose2d(
                    7.778, FieldConstants.kFieldSize.getY() - 5.452,
                    Rotation2d.fromDegrees(69.395)
                ),
                new Pose2d(
                    6.187, FieldConstants.kFieldSize.getY() - 7.108,
                    Rotation2d.fromDegrees(-156.938)
                )
            ),
            new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
            // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(TrenchAlignConstants.kStage2Speed, Rotation2d.fromDegrees(0))
        ),
        new TrenchAlign()
    );
}
