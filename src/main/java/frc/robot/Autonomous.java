package frc.robot;

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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.AutoDriveCommand;
import frc.robot.commands.TrenchAlign;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.FieldConstants.TrenchLocations;
import frc.robot.utils.Constants.TrenchAlignConstants;

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
                new PIDConstants(5, 0, 0)
            ),
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
                        Rotation2d.fromDegrees(-180)
                    )
                ),
                new PathConstraints(.5, 1, 3 * Math.PI, 4 * Math.PI),
                new IdealStartingState(0, Rotation2d.fromDegrees(-90)),
                new GoalEndState(TrenchAlignConstants.kStage1Speed, Rotation2d.fromDegrees(-180))
            ),
            new TrenchAlign(),
            new AutoDriveCommand(
                List.of(
                    new Pose2d(
                        TrenchLocations.kBlueRightCenter.getX() - 1.2,
                        TrenchLocations.kBlueRightCenter.getY(), Rotation2d.fromDegrees(-180)
                    ),
                    new Pose2d(0.686, 0.680, Rotation2d.fromDegrees(0))
                ),
                new PathConstraints(.5, 1, 3 * Math.PI, 4 * Math.PI),
                new IdealStartingState(TrenchAlignConstants.kStage2Speed, Rotation2d.fromDegrees(180)),
                new GoalEndState(0, Rotation2d.fromDegrees(-180))
            ));

        Command midOutpostAuto = new SequentialCommandGroup(
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

        Command rightChoateAuto = new SequentialCommandGroup(
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
                    new EventMarker("Stop Snake Drive", 1, new InstantCommand(() -> stopSnakeDrive()))
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

        autoChooser.addOption("R - Mid/Left Climb Auto", rightMidLeftClimbAuto);
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