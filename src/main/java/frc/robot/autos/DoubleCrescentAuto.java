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
import frc.robot.commands.OutpostAlign;
import frc.robot.commands.TrenchAlign;
import frc.robot.commands.TrenchAlignAutonomous;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.FieldConstants.TrenchLocations;
import frc.robot.utils.Constants.TrenchAlignConstants;

public class DoubleCrescentAuto {
    private static final double maxSpeed = 3;
    private static final double maxAcceleration = 3;
    public static final Command auto = new SequentialCommandGroup(
        new InstantCommand(() -> {
            Drivetrain.getInstance().setStartingPose(new Pose2d(4.453, FieldConstants.kFieldSize.getY() - 7.522, Rotation2d.fromDegrees(0)));
        }),
        new AutoDriveCommand(
            List.of(
                new Pose2d(4.453, FieldConstants.kFieldSize.getY() - 7.522, Rotation2d.fromDegrees(0)),
                new Pose2d(5.8, FieldConstants.kFieldSize.getY() - 7.522, Rotation2d.fromDegrees(0))
            ),
            new PathConstraints(maxSpeed, maxAcceleration, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(0, Rotation2d.fromDegrees(0)),
            new GoalEndState(2, Rotation2d.fromDegrees(0))
            ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(5.8, FieldConstants.kFieldSize.getY() - 7.522, Rotation2d.fromDegrees(0)),
                new Pose2d(7.5, FieldConstants.kFieldSize.getY() - 6.370, Rotation2d.fromDegrees(65))
            ),
            new PathConstraints(maxSpeed, maxAcceleration, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(1, Rotation2d.fromDegrees(0)),
            new GoalEndState(2, Rotation2d.fromDegrees(90))
        ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(7.5, FieldConstants.kFieldSize.getY() - 6.370, Rotation2d.fromDegrees(65)),
                new Pose2d(7.5, 5.309, Rotation2d.fromDegrees(90)),
                new Pose2d(6.614, 6.720, Rotation2d.fromDegrees(150))
            ),
            new PathConstraints(1.5, 1.5, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(2, Rotation2d.fromDegrees(90)),
            new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(90))
        ),

        new TrenchAlignAutonomous(0, true),
        new InstantCommand(() -> Drivetrain.getInstance().drive(new Translation2d(0, 0), 0, true, new Translation2d(0, 0))),
        new WaitCommand(3.5),
        new TrenchAlignAutonomous(0, true),

        new AutoDriveCommand(
            List.of(
                new Pose2d(TrenchLocations.kBlueLeftCenter.plus(TrenchAlignConstants.kOutOffset).getX(), TrenchLocations.kBlueLeftCenter.getY(), Rotation2d.fromDegrees(0)),
                new Pose2d(5.9, TrenchLocations.kBlueLeftCenter.getY(), Rotation2d.fromDegrees(0))
            ),
            new PathConstraints(maxSpeed, maxAcceleration, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(2, Rotation2d.fromDegrees(0)),
            new GoalEndState(2, Rotation2d.fromDegrees(0))
            ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(5.9, TrenchLocations.kBlueLeftCenter.getY(), Rotation2d.fromDegrees(0)),
                new Pose2d(6.25, 6.720, Rotation2d.fromDegrees(-65))
            ),
            new PathConstraints(maxSpeed, maxAcceleration, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(2, Rotation2d.fromDegrees(0)),
            new GoalEndState(2, Rotation2d.fromDegrees(-90))
        ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(6.25, 6.720, Rotation2d.fromDegrees(-90)),
                new Pose2d(6.25, 3, Rotation2d.fromDegrees(-90))
            ),
            new PathConstraints(1.5, 1.5, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(2, Rotation2d.fromDegrees(-90)),
            new GoalEndState(2, Rotation2d.fromDegrees(-90))
        ),
        new AutoDriveCommand(
            List.of(
                new Pose2d(6.25, 3, Rotation2d.fromDegrees(-90)),
                new Pose2d(6.25, 1.997, Rotation2d.fromDegrees(-90))
            ),
            new PathConstraints(maxSpeed, maxAcceleration, 3 * Math.PI, 4 * Math.PI),
            new IdealStartingState(2, Rotation2d.fromDegrees(-90)),
            new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(-30)) //pre turn
        ),
        
        new TrenchAlignAutonomous(0, true),
        new InstantCommand(() -> Drivetrain.getInstance().drive(new Translation2d(0, 0), 0, true, new Translation2d(0, 0))),
        new WaitCommand(3.5) //may remove and just shoot all at the outpost
        // new OutpostAlign()
    );
}