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
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Autonomous;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.OutpostAlign;
import frc.robot.commands.TrenchAlign;
import frc.robot.commands.TrenchAlignIntakeFirst;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.FieldConstants.TrenchLocations;
import frc.robot.utils.Constants.TrenchAlignConstants;

public class MidOutpostAuto {
    private static final double maxSpeed = 3;
    private static final double maxAcceleration = 3;
    public static final Command auto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            Drivetrain.getInstance().setStartingPose(new Pose2d(4.453, 7.522, Rotation2d.fromDegrees(0)));
        }),
        new AutoDriveCommand(
            List.of(
                new Pose2d(4.453, 7.522, Rotation2d.fromDegrees(0)),
                new Pose2d(5.8, 7.522, Rotation2d.fromDegrees(0))
            ),
            new PathConstraints(maxSpeed, maxAcceleration, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(3, Rotation2d.fromDegrees(0))
            ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(5.8, 7.522, Rotation2d.fromDegrees(0)),
                new Pose2d(7.093, 6.720, Rotation2d.fromDegrees(-90))
            ),
            new PathConstraints(maxSpeed, maxAcceleration, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(3, Rotation2d.fromDegrees(0)),
            new GoalEndState(3, Rotation2d.fromDegrees(-90))
        ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(7.093, 6.720, Rotation2d.fromDegrees(-90)),
                new Pose2d(7.093, 6.025, Rotation2d.fromDegrees(-90))
            ),
            new PathConstraints(maxSpeed, maxAcceleration, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(3, Rotation2d.fromDegrees(-90)),
            new GoalEndState(3, Rotation2d.fromDegrees(-90))
            ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(7.093, 6.025, Rotation2d.fromDegrees(-90)),
                new Pose2d(7.093, 1.524, Rotation2d.fromDegrees(-90))
            ),
            new PathConstraints(maxSpeed, maxAcceleration, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(3, Rotation2d.fromDegrees(-90)),
            new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(-90))
        ),

        new TrenchAlignIntakeFirst(),

        new OutpostAlign()
    );
}