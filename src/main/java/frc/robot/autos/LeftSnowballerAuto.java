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

public class LeftSnowballerAuto {
    public static final Command auto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            Drivetrain.getInstance().setStartingPose(new Pose2d(4.466, 7.367, Rotation2d.fromDegrees(0)));
            //Autonomous.startSnakeDrive();
            Autonomous.startPassDrive();
        }),
        new AutoDriveCommand(
            List.of(
                new Pose2d(4.466, 7.367, Rotation2d.fromDegrees(0)),
                new Pose2d(7.478, 6.385, Rotation2d.fromDegrees(-66.501)),
                new Pose2d(7.478, 1.400, Rotation2d.fromDegrees(96.802)),
                new Pose2d(7.59, 6.418, Rotation2d.fromDegrees(-57.195)),
                new Pose2d(7.311, 1.400, Rotation2d.fromDegrees(107.882)),
                new Pose2d(7.478, 6.418, Rotation2d.fromDegrees(61.366)),
                new Pose2d(7.286, 2.128, Rotation2d.fromDegrees(-102.907))
            ),
            List.of(
                new EventMarker("Stop Passing", 5.67)
            ),
            new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(180))
        ),
        new TrenchAlign(true)
    );
}
