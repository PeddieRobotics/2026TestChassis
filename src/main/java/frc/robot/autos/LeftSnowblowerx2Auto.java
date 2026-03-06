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
            Autonomous.startSnakeDrive();
            Autonomous.startPassDrive();
        }),
        new AutoDriveCommand(
            List.of(
                new Pose2d(4.466, 7.367, Rotation2d.fromDegrees(0)),
                new Pose2d(6.778, 6.385, Rotation2d.fromDegrees(-66.501)),
                new Pose2d(6.778, 1.400, Rotation2d.fromDegrees(96.802)),
                new Pose2d(6.89, 6.418, Rotation2d.fromDegrees(-151.432)),
                new Pose2d(6.89, 1.400, Rotation2d.fromDegrees(-96.233))
            ),
            List.of(
                new RotationTarget(1.4, Rotation2d.fromDegrees(-90)),
                new RotationTarget(2.4, Rotation2d.fromDegrees(90)),
                new RotationTarget(3.4, Rotation2d.fromDegrees(-90))
            ),
            List.of(
                //new EventMarker("Stop Passing", 5.67)
            ),
            new PathConstraints(0.75, 3, 5 * Math.PI, 6 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(-91))
        ),
        new TrenchAlign(true)
    );
}
