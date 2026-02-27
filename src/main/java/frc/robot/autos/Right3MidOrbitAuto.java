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
import com.pathplanner.lib.path.RotationTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
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

public class Right3MidOrbitAuto {
    public static final Command auto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            Drivetrain.getInstance().setStartingPose(new Pose2d(
                4.466, FieldConstants.kFieldSize.getY() - 7.406,
                Rotation2d.fromDegrees(0)
            ));
        }),
        new AutoDriveCommand(
            List.of(
                new Pose2d(
                    4.466, FieldConstants.kFieldSize.getY() - 7.406,
                    Rotation2d.fromDegrees(0)
                ),
                new Pose2d(
                    7.975, FieldConstants.kFieldSize.getY() - 6.264,
                    Rotation2d.fromDegrees(71.274)
                )
            ),
            List.of(
                new RotationTarget(0.8, Rotation2d.fromDegrees(90))
            ),
            List.of(
                // new EventMarker("Face LEFT", 0.65, new InstantCommand(() -> Rotation2d.fromDegrees(90)))
            ),
            new PathConstraints(1.5, 1.5, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(89))
        ),
        new TrenchAlign(true),
        new InstantCommand(() -> Drivetrain.getInstance().drive(new Translation2d(0, 0), 0, true, new Translation2d(0, 0))),
        new WaitCommand(2.5), // !!TODO: change after the wrenchers are ready
        new TrenchAlign(false),
        new AutoDriveCommand(
            List.of(
                new Pose2d(
                    TrenchLocations.kBlueRightCenter.plus(TrenchLocations.kOffset),
                    Rotation2d.fromDegrees(180)
                ),
                new Pose2d(
                    6.778, FieldConstants.kFieldSize.getY() - 5.452,
                    Rotation2d.fromDegrees(-133.697)
                )
                // new Pose2d(
                //     6.187, FieldConstants.kFieldSize.getY() - 7.108,
                //     Rotation2d.fromDegrees(-156.938)
                // )
            ),
            List.of(
                new RotationTarget(0.8, Rotation2d.fromDegrees(90))
            ),
            List.of(
                // new EventMarker("Face LEFT", 0.92, new InstantCommand(() -> Rotation2d.fromDegrees(90)))
            ),
            new PathConstraints(2, 2, 3 * Math.PI, 4 * Math.PI),
            // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
            new IdealStartingState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(0)),
            new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(80))
        ),
        new TrenchAlign(true),
        new InstantCommand(() -> Drivetrain.getInstance().drive(new Translation2d(0, 0), 0, true, new Translation2d(0, 0))),
        new WaitCommand(2.5), // !!TODO: change after the wrenchers are ready
        new TrenchAlign(false), //!!FIXXX
        new AutoDriveCommand(
            List.of(
                new Pose2d(
                    TrenchLocations.kBlueRightCenter.plus(TrenchLocations.kOffset),
                    Rotation2d.fromDegrees(180)
                ),
                new Pose2d(
                    7.183, FieldConstants.kFieldSize.getY() - 7.406,
                    Rotation2d.fromDegrees(0)
                )
            ),
            List.of(
                //new RotationTarget(0.8, Rotation2d.fromDegrees(90))
            ),
            List.of(
                // new EventMarker("Face LEFT", 0.99, new InstantCommand(() -> Rotation2d.fromDegrees(90)))
            ),
            new PathConstraints(1.5, 1.5, 3 * Math.PI, 4 * Math.PI),
            // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
            new IdealStartingState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(0)),
            new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(0))
            //new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(89))
        ),
        new TrenchAlign(true)
    );
}
