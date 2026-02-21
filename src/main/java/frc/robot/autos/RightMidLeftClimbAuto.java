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

public class RightMidLeftClimbAuto {
    public static final Command auto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            Drivetrain.getInstance().setStartingPose(new Pose2d(4.453, 0.664, Rotation2d.fromDegrees(0)));
            Autonomous.startSnakeDrive();
        }),
        new AutoDriveCommand(
            List.of(
                new Pose2d(4.453, 0.664, Rotation2d.fromDegrees(0)),
                new Pose2d(7.856, 1.764, Rotation2d.fromDegrees(87.207)),
                new Pose2d(7.532, 6.590, Rotation2d.fromDegrees(136.668)),
                new Pose2d(6.342, 7.315, Rotation2d.fromDegrees(173.723))
            ),
            new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(0.5, Rotation2d.fromDegrees(180))
        ),
        new TrenchAlign()
    // new AutoDriveCommand(
    // List.of(
    // new
    // Pose2d(TrenchLocations.kBlueLeftCenter.minus(TrenchAlignConstants.kOutOffset),
    // Rotation2d.fromDegrees(-167.074)),
    // new Pose2d(1.180, 4.870, Rotation2d.fromDegrees(-135.999))
    // ),
    // new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
    // new IdealStartingState(TrenchAlignConstants.kStage2Speed,
    // Rotation2d.fromDegrees(180)),
    // new GoalEndState(0, Rotation2d.fromDegrees(0))
    // )
    );
}
