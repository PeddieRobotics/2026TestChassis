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

public class MidOutpostAuto {
    public static final Command auto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            Drivetrain.getInstance().setStartingPose(new Pose2d(4.427, 7.297, Rotation2d.fromDegrees(0)));
        }),
        new AutoDriveCommand(
            List.of(
                new Pose2d(4.427, 7.297, Rotation2d.fromDegrees(0)),
                new Pose2d(6.390, 7.297, Rotation2d.fromDegrees(0))
            ),
            new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(0, Rotation2d.fromDegrees(0))
        ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(6.390, 7.297, Rotation2d.fromDegrees(0)),
                new Pose2d(7.802, 6.025, Rotation2d.fromDegrees(-90.57))
            ),
            new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(0, Rotation2d.fromDegrees(-90))
        ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(7.802, 6.025, Rotation2d.fromDegrees(-90.57)),
                new Pose2d(7.090, 0.88, Rotation2d.fromDegrees(-169.266)),
                new Pose2d(
                    TrenchLocations.kBlueRightCenter.getX() + TrenchLocations.kCloseOffset.getX(),
                    TrenchLocations.kBlueRightCenter.getY(), Rotation2d.fromDegrees(180)
                )
            ),
            new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(-90)),
            new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(180))
        ),

        new TrenchAlign(),

        new AutoDriveCommand(
            List.of(
                new Pose2d(
                    TrenchLocations.kBlueRightCenter.getX() - TrenchLocations.kCloseOffset.getX(),
                    TrenchLocations.kBlueRightCenter.getY(), Rotation2d.fromDegrees(180)
                ),
                new Pose2d(0.610, 0.680, Rotation2d.fromDegrees(180))
            ),
            new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(TrenchAlignConstants.kStage2Speed, Rotation2d.fromDegrees(-90)),
            new GoalEndState(0, Rotation2d.fromDegrees(180))
        )
    );
}