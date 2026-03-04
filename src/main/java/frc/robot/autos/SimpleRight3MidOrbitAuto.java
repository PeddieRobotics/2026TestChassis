package frc.robot.autos;

import java.lang.reflect.Field;
import java.util.List;

import javax.print.attribute.standard.Fidelity;

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

public class SimpleRight3MidOrbitAuto {
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
                    5.820, FieldConstants.kFieldSize.getY() - 7.406,
                    Rotation2d.fromDegrees(0)
                )
            ),
            new PathConstraints(1.5, 1.5, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(1.5, Rotation2d.fromDegrees(0))
        ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(
                    5.570, FieldConstants.kFieldSize.getY() - 7.406,
                    Rotation2d.fromDegrees(0)
                ),
                new Pose2d(
                    7.475, FieldConstants.kFieldSize.getY() - 6.264,
                    Rotation2d.fromDegrees(90)
                )
            ),
            new PathConstraints(1.5, 1.5, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(1.5, Rotation2d.fromDegrees(90))
        ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(
                    7.475, FieldConstants.kFieldSize.getY() - 6.264,
                    Rotation2d.fromDegrees(90)
                ),
                new Pose2d(
                    7.475, FieldConstants.kFieldSize.getY() - 4.264,
                    Rotation2d.fromDegrees(90)
                ),
                new Pose2d(
                    7.475, FieldConstants.kFieldSize.getY() - 6.264,
                    Rotation2d.fromDegrees(-90)
                )
            ),
            new PathConstraints(1.5, 1.5, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(1.5, Rotation2d.fromDegrees(90)),
            new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(85))
        ),
        new TrenchAlign(true),
        new InstantCommand(() -> Drivetrain.getInstance().drive(new Translation2d(0, 0), 0, true, new Translation2d(0, 0))),
        new WaitCommand(3),
        new TrenchAlign(true),
        new AutoDriveCommand(
            List.of(
                new Pose2d(
                    TrenchLocations.kBlueRightCenter.plus(TrenchAlignConstants.kOutOffset),
                    Rotation2d.fromDegrees(0)
                ),
                new Pose2d(
                    6.778, FieldConstants.kFieldSize.getY() - 5.452,
                    Rotation2d.fromDegrees(90)
                )
            ),
            new PathConstraints(1.5, 1.5, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(0)),
            new GoalEndState(1.5, Rotation2d.fromDegrees(90))
        ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(
                    6.778, FieldConstants.kFieldSize.getY() - 5.452,
                    Rotation2d.fromDegrees(90)
                ),
                new Pose2d(
                    6.778, FieldConstants.kFieldSize.getY() - 4.452,
                    Rotation2d.fromDegrees(90)
                ),
                new Pose2d(
                    6.778, FieldConstants.kFieldSize.getY() - 5.452,
                    Rotation2d.fromDegrees(-90)
                )
            ),
            new PathConstraints(1.5, 1.5, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(1.5, Rotation2d.fromDegrees(90)),
            new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(85))
        ),
        new TrenchAlign(true),
        new InstantCommand(() -> Drivetrain.getInstance().drive(new Translation2d(0, 0), 0, true, new Translation2d(0, 0))),
        new WaitCommand(3),
        new TrenchAlign(true)
    );
}
