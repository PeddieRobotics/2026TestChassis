package frc.robot;

import static edu.wpi.first.units.Units.Rotation;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.events.Event;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPoint;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.networktables.TopicInfo;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.TrenchAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.FieldConstants.TrenchLocations;
import frc.robot.utils.Constants.TrenchAlignConstants;
import frc.robot.commands.TrenchAlign.TrenchOption;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.TrenchAlignConstants;
import frc.robot.utils.Constants.FieldConstants.TrenchLocations;


public class Autonomous {

    private static Autonomous autonomous;
    private SendableChooser<Command> autoChooser;
    private SendableChooser<Double> autoStartPosition;

    public static Autonomous getInstance() {
        if (autonomous == null)
            autonomous = new Autonomous();
        return autonomous;
    }

    RobotConfig config;

    private void startSnakeDrive() {
        PPHolonomicDriveController.overrideRotationFeedback(() -> {
            return Drivetrain.getInstance().getRotationOverride();
        });
    }

    private void stopSnakeDrive() {
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }

    private void startPassDrive() {
        // TODO!!!! implement passDrive
    }

    private void stopPassDrive() {
        // TODO!!! implement stop passDrive
    }

    private Autonomous() {
        Drivetrain drivetrain = Drivetrain.getInstance();

        // config = new RobotConfig(
        // 74.088,
        // 6.883,
        // new ModuleConfig(0.048, 5.450, 1.200, DCMotor.getKrakenX60(1), 60, 1),
        // new Translation2d(0.273, 0.273),
        // new Translation2d(0.273, -0.273),
        // new Translation2d(-0.273, 0.273),
        // new Translation2d(-0.273, -0.273)
        // );

        try {
            config = RobotConfig.fromGUISettings();
        } catch (Exception e) {
            e.printStackTrace();
        }

        AutoBuilder.configure(
                drivetrain::getPose,
                drivetrain::setPose,
                drivetrain::getRobotRelativeSpeeds,
                (speeds, feedforwards) -> drivetrain.driveRobotRelative(speeds),
                new PPHolonomicDriveController(
                        new PIDConstants(5, 0, 0),
                        new PIDConstants(5, 0, 0)),
                config,
                () -> {
                    var alliance = DriverStation.getAlliance();
                    if (alliance.isPresent()) {
                        return alliance.get() == DriverStation.Alliance.Red;
                    }
                    return false;
                },
                drivetrain // Reference to this subsystem to set requirements
        );

        autoChooser = new SendableChooser<>();

        Command trenchTestAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(7.855, 2.92, Rotation2d.fromDegrees(-90)));
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(7.855, 2.92, Rotation2d.fromDegrees(-90)),
                                new Pose2d(
                                    TrenchLocations.kBlueRightCenter.plus(new Translation2d(1.6, 0)),
                                    Rotation2d.fromDegrees(-180))),
                        new PathConstraints(.5, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(-90)),
                        new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(-180))),
                new TrenchAlign(),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(
                                        TrenchLocations.kBlueRightCenter.getX() - 1.2,
                                        TrenchLocations.kBlueRightCenter.getY(), Rotation2d.fromDegrees(-180)),
                                new Pose2d(0.686, 0.680, Rotation2d.fromDegrees(0))),
                        new PathConstraints(.5, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(TrenchAlignConstants.kStage2Speed, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-180)))
            );

        Command midOutpostAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(4.427, 7.297, Rotation2d.fromDegrees(0)));
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(4.427, 7.297, Rotation2d.fromDegrees(0)),
                                new Pose2d(6.390, 7.297, Rotation2d.fromDegrees(0))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                        new GoalEndState(0, Rotation2d.fromDegrees(0))),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(6.390, 7.297, Rotation2d.fromDegrees(0)),
                                new Pose2d(7.802, 6.025, Rotation2d.fromDegrees(-90.57))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-90))),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(7.802, 6.025, Rotation2d.fromDegrees(-90.57)),
                                new Pose2d(7.090, 0.88, Rotation2d.fromDegrees(-169.266)),
                                new Pose2d(
                                        TrenchLocations.kBlueRightCenter.getX() + TrenchLocations.kCloseOffset.getX(),
                                        TrenchLocations.kBlueRightCenter.getY(), Rotation2d.fromDegrees(180))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(-90)),
                        new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(180))),

                new TrenchAlign(),

                new AutoDriveCommand(
                        List.of(
                                new Pose2d(
                                        TrenchLocations.kBlueRightCenter.getX() - TrenchLocations.kCloseOffset.getX(),
                                        TrenchLocations.kBlueRightCenter.getY(), Rotation2d.fromDegrees(180)),
                                new Pose2d(0.610, 0.680, Rotation2d.fromDegrees(180))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(TrenchAlignConstants.kStage2Speed, Rotation2d.fromDegrees(-90)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180)))

        );




    Command rightMidLeftClimbAuto = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(4.453, 0.664, Rotation2d.fromDegrees(0)));
                startSnakeDrive();
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
            //     List.of(
            //         new Pose2d(TrenchLocations.kBlueLeftCenter.minus(TrenchAlignConstants.kOutOffset), Rotation2d.fromDegrees(-167.074)),
            //         new Pose2d(1.180, 4.870, Rotation2d.fromDegrees(-135.999))
            //     ),
            //     new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
            //     new IdealStartingState(TrenchAlignConstants.kStage2Speed, Rotation2d.fromDegrees(180)),
            //     new GoalEndState(0, Rotation2d.fromDegrees(0))
            // )
        );

        Command choateAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(4.505, 7.587, Rotation2d.fromDegrees(16.928)));
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(4.505, 7.587, Rotation2d.fromDegrees(16.928)),
                                new Pose2d(8.605, 5.594, Rotation2d.fromDegrees(113.485)),
                                new Pose2d(4.893, 7.406, Rotation2d.fromDegrees(-178.493)),
                                new Pose2d(3.470, 7.406, Rotation2d.fromDegrees(-4.764)),
                                new Pose2d(6.762, 7.270, Rotation2d.fromDegrees(-6.116)),
                                new Pose2d(8.037, 4.378, Rotation2d.fromDegrees(-69.395)),
                                new Pose2d(5.636, 7.406, Rotation2d.fromDegrees(174.336)),
                                new Pose2d(2.926, 7.056, Rotation2d.fromDegrees(-143.276)),
                                new Pose2d(1.620, 4.378, Rotation2d.fromDegrees(-129.174))),
                        List.of(
                                new RotationTarget(1.87, Rotation2d.fromDegrees(-90)),
                                new RotationTarget(2.3, Rotation2d.fromDegrees(-90)),
                                new RotationTarget(3.61, Rotation2d.fromDegrees(-90)),
                                new RotationTarget(5.9, Rotation2d.fromDegrees(-90)),
                                new RotationTarget(6.66, Rotation2d.fromDegrees(-90))),
                        List.of(
                                new EventMarker("Stop Snake Drive", 0.94),
                                new EventMarker("Start Snake Drive", 4.17),
                                new EventMarker("Stop Snake Drive", 5.23)),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(16.928)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-129.174))));


        // TODO: resolve these
        Command rightChoateAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(
                            new Pose2d(4.505, FieldConstants.kFieldSize.getY() - 7.587, Rotation2d.fromDegrees(0)));
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(4.505, FieldConstants.kFieldSize.getY() - 7.587, Rotation2d.fromDegrees(0)),
                                new Pose2d(8.019, FieldConstants.kFieldSize.getY() - 6.057,
                                        Rotation2d.fromDegrees(-122.095)),
                                new Pose2d(4.893, FieldConstants.kFieldSize.getY() - 7.406,
                                        Rotation2d.fromDegrees(178.493)),
                                new Pose2d(3.470, FieldConstants.kFieldSize.getY() - 7.406,
                                        Rotation2d.fromDegrees(4.764)),
                                new Pose2d(6.762, FieldConstants.kFieldSize.getY() - 7.270,
                                        Rotation2d.fromDegrees(6.116)),
                                new Pose2d(7.833, FieldConstants.kFieldSize.getY() - 5.15,
                                        Rotation2d.fromDegrees(69.395)),
                                new Pose2d(5.636, FieldConstants.kFieldSize.getY() - 7.406,
                                        Rotation2d.fromDegrees(-174.336)),
                                new Pose2d(2.926, FieldConstants.kFieldSize.getY() - 7.056,
                                        Rotation2d.fromDegrees(143.276)),
                                new Pose2d(1.620, FieldConstants.kFieldSize.getY() - 5.878,
                                        Rotation2d.fromDegrees(129.174))),
                        List.of(
                                new RotationTarget(1.87, Rotation2d.fromDegrees(90)),
                                new RotationTarget(2.3, Rotation2d.fromDegrees(90)),
                                new RotationTarget(3.61, Rotation2d.fromDegrees(90)),
                                new RotationTarget(5.9, Rotation2d.fromDegrees(90)),
                                new RotationTarget(6.66, Rotation2d.fromDegrees(90))),
                        List.of(
                        // new EventMarker("Stop Snake Drive", 0.94),
                        // new EventMarker("Start Snake Drive", 4.17),
                        // new EventMarker("Stop Snake Drive", 5.23)
                        ),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                        new GoalEndState(0, Rotation2d.fromDegrees(90))));
        Command rightChoateAuto2 = new SequentialCommandGroup(
            new InstantCommand(() -> {
                Drivetrain.getInstance().setStartingPose(new Pose2d(4.466, FieldConstants.kFieldSize.getY() - 7.406, Rotation2d.fromDegrees(-10.284)));
            }),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(4.466, FieldConstants.kFieldSize.getY() - 7.406, Rotation2d.fromDegrees(-10.284)), 
                    new Pose2d(7.975, FieldConstants.kFieldSize.getY() - 6.264, Rotation2d.fromDegrees(133.132)),
                    new Pose2d(6.329, FieldConstants.kFieldSize.getY() - 7.108, Rotation2d.fromDegrees(-148.512))
                ),

                List.of(
                    new EventMarker("Stop Snake Drive", 1, new InstantCommand(() -> stopSnakeDrive()))
                ),
                new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                new GoalEndState(TrenchAlignConstants.kStage2Speed, Rotation2d.fromDegrees(0))
            ),
            new TrenchAlign(TrenchOption.RIGHT),
            new WaitCommand(1), //!!TODO: change after the wrenchers are ready
            new TrenchAlign(TrenchOption.RIGHT),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(TrenchLocations.kRedRightCenter.plus(TrenchAlignConstants.kOutOffset), Rotation2d.fromDegrees(180)), 
                    new Pose2d(7.778, FieldConstants.kFieldSize.getY() - 5.452, Rotation2d.fromDegrees(69.395)),
                    new Pose2d(6.187, FieldConstants.kFieldSize.getY() - 7.108, Rotation2d.fromDegrees(-156.938))
                ),
                new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                new GoalEndState(TrenchAlignConstants.kStage2Speed, Rotation2d.fromDegrees(0))
            ),
            new TrenchAlign(TrenchOption.RIGHT)
        );

        autoChooser.addOption("R - Mid/Left Climb Auto", rightMidLeftClimbAuto);
        autoChooser.addOption("Left Choate Trench Auto", choateAuto);
        autoChooser.addOption("Right Choate Trench Auto", rightChoateAuto);
        autoChooser.addOption("Left Mid Outpost Auto", midOutpostAuto);
        autoChooser.addOption("Left Trench Test Auto", trenchTestAuto);

        // autoChooser.setDefaultOption("Outpost", outpostAuto);
        // autoChooser.setDefaultOption("Squiggle", squiggleAuto);
        // autoChooser.setDefaultOption("Not Squiggle", notsosquiggleAuto);

        SmartDashboard.putData("Auto Routines", autoChooser);

        autoStartPosition = new SendableChooser<>();
        autoStartPosition.setDefaultOption("1anywhere", 0.0);

        SmartDashboard.putData("Auto Starting Position", autoStartPosition);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

// rightMidLeftClimbAuto
// midOutpost
// trenchTest
// rightChoateauto ??