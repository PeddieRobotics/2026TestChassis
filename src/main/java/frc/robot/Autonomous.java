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

        Command autoCommand = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(3.586, 0.820, Rotation2d.fromDegrees(0)));
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(3.586, 0.820, Rotation2d.fromDegrees(0)),
                                new Pose2d(2.305, 0.820, Rotation2d.fromDegrees(0))),
                        new PathConstraints(0.5, 0.5, Math.PI, Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                        new GoalEndState(0, Rotation2d.fromDegrees(0)))

        );

        Command basicRightOutpostAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)));
                    // startSnakeDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)),
                                new Pose2d(0.856, 0.639, Rotation2d.fromDegrees(180.000))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))),

                new WaitCommand(2),

                new AutoDriveCommand(
                        List.of(
                                new Pose2d(0.856, 0.639, Rotation2d.fromDegrees(180.000)),
                                new Pose2d(2.564, 2.761, Rotation2d.fromDegrees(71.696))),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))));

        // TODO CHECK START AND END STATES

        Command rightOutDepClimbLAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)));
                    // startSnakeDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)),
                                new Pose2d(0.856, 0.639, Rotation2d.fromDegrees(180.000))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))),

                new WaitCommand(2),

                new AutoDriveCommand(
                        List.of(
                                new Pose2d(0.856, 0.639, Rotation2d.fromDegrees(180.000)),
                                new Pose2d(2.163, 3.653, Rotation2d.fromDegrees(71.696)),
                                new Pose2d(2.163, 5.918, Rotation2d.fromDegrees(180)),
                                new Pose2d(1.500, 5.918, Rotation2d.fromDegrees(0))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(0))),

                new WaitCommand(0.5),

                new AutoDriveCommand(
                        List.of(
                                new Pose2d(1.500, 5.918, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.063, 4.688, Rotation2d.fromDegrees(-74.055))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-74.055)))

        );

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

        Command leftMidOutClimbRAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(4.427, 7.297, Rotation2d.fromDegrees(0)));
                    startSnakeDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(4.427, 7.297, Rotation2d.fromDegrees(0)),
                                new Pose2d(7.802, 6.025, Rotation2d.fromDegrees(-90.57)),
                                new Pose2d(7.090, 0.88, Rotation2d.fromDegrees(-169.266)),
                                new Pose2d(1.048, 0.622, Rotation2d.fromDegrees(180))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))),

                new WaitCommand(2),

                new AutoDriveCommand(
                        List.of(
                                new Pose2d(1.048, 0.622, Rotation2d.fromDegrees(180)),
                                new Pose2d(1.048, 3.048, Rotation2d.fromDegrees(114.544))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(114.544))));

        Command leftMidPassScoreClimbLAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(4.466, 7.367, Rotation2d.fromDegrees(0)));
                    startSnakeDrive();
                    startPassDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(4.427, 7.297, Rotation2d.fromDegrees(0)),
                                new Pose2d(7.636, 6.448, Rotation2d.fromDegrees(-66.501)),
                                new Pose2d(7.878, 1.324, Rotation2d.fromDegrees(-71.147))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-71.147))),
                // TODO: add turn 180 command!!
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(8.024, 6.616, Rotation2d.fromDegrees(117.255)),
                                new Pose2d(4.466, 7.367, Rotation2d.fromDegrees(179.409)),
                                new Pose2d(1.05, 4.675, Rotation2d.fromDegrees(-112.380))),
                        List.of(
                                new EventMarker("Stop Snake Drive", 4, new InstantCommand(() -> stopSnakeDrive())),
                                new EventMarker("Stop Passing", 2.07, new InstantCommand(() -> stopPassDrive()))

                        ),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(117.255)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-112.380))));

        Command rightMidPassScoreClimbLAuto = new SequentialCommandGroup( /// 2/17 TEST!!!!
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(
                            new Pose2d(4.466, FieldConstants.kFieldSize.getY() - 7.367, Rotation2d.fromDegrees(0)));
                    startSnakeDrive();
                    stopPassDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(4.427, FieldConstants.kFieldSize.getY() - 7.367, Rotation2d.fromDegrees(0)),
                                new Pose2d(7.636, FieldConstants.kFieldSize.getY() - 6.448,
                                        Rotation2d.fromDegrees(66.501)),
                                new Pose2d(7.878, FieldConstants.kFieldSize.getY() - 1.324,
                                        Rotation2d.fromDegrees(71.147))),
                        new PathConstraints(1, 1, Math.PI, Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                        new GoalEndState(0, Rotation2d.fromDegrees(71.147))),
                // TODO: add turn 180 command?
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(8.024, FieldConstants.kFieldSize.getY() - 6.616,
                                        Rotation2d.fromDegrees(-117.255)),
                                new Pose2d(4.466, FieldConstants.kFieldSize.getY() - 7.367,
                                        Rotation2d.fromDegrees(-179.409)),
                                new Pose2d(1.05,
                                        FieldConstants.kFieldSize.getY() - 4.675 - FieldConstants.kFieldTowerAutoOffset,
                                        Rotation2d.fromDegrees(112.380))),
                        List.of(
                                new EventMarker("Stop Snake Drive", 4, new InstantCommand(() -> stopSnakeDrive())),
                                new EventMarker("Stop Passing", 2.07, new InstantCommand(() -> stopPassDrive()))

                        ),
                        new PathConstraints(1, 1, Math.PI, Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(-117.255)),
                        new GoalEndState(0, Rotation2d.fromDegrees(112.380))));

        Command leftDepOutClimbRAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(3.560, 5.853, Rotation2d.fromDegrees(180)));
                    // startSnakeDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(3.560, 5.853, Rotation2d.fromDegrees(180)),
                                new Pose2d(1.400, 5.950, Rotation2d.fromDegrees(0))),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),

                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))),
                new WaitCommand(0.5),

                new AutoDriveCommand(
                        List.of(
                                new Pose2d(1.400, 5.950, Rotation2d.fromDegrees(0)),
                                new Pose2d(2.254, 2.903, Rotation2d.fromDegrees(-100.222)),
                                new Pose2d(0.649, 0.833, Rotation2d.fromDegrees(90))

                        ),
                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),

                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))),
                new WaitCommand(2),

                new AutoDriveCommand(
                        List.of(
                                new Pose2d(0.649, 0.833, Rotation2d.fromDegrees(90)),
                                new Pose2d(1.063, 2.55, Rotation2d.fromDegrees(93.764))),

                        List.of(
                                new RotationTarget(3.5, Rotation2d.fromDegrees(180))),

                        List.of(
                                new EventMarker("", 0.0)),

                        new PathConstraints(1, 1, 3 * Math.PI, 4 * Math.PI),

                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))

                ));

        Command crazyWeirdAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(180)));
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(180)),
                                new Pose2d(2.034, 2.541, Rotation2d.fromDegrees(91.449)),
                                new Pose2d(2.771, 3.912, Rotation2d.fromDegrees(81.251)),
                                new Pose2d(2.304, 5.297, Rotation2d.fromDegrees(77.259))),
                        List.of(
                        // add rotation target / holonomic rotations code!!!
                        ),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-90)))

        );

        Command rightMidLeftClimbAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(4.453, 0.664, Rotation2d.fromDegrees(180)));
                    startSnakeDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(4.453, 0.664, Rotation2d.fromDegrees(180)),
                                new Pose2d(7.752, 1.816, Rotation2d.fromDegrees(91.273)),
                                new Pose2d(7.196, 6.862, Rotation2d.fromDegrees(154.898)),
                                new Pose2d(4.453, 7.496, Rotation2d.fromDegrees(179.246)),
                                new Pose2d(1.000, 4.637, Rotation2d.fromDegrees(-117.714))),
                        List.of(
                                new EventMarker("Stop Snake Drive", 3.15, new InstantCommand(() -> stopSnakeDrive()))),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-117.714)))

        );

        Command leftMidStoreDepotAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(4.453, 7.38, Rotation2d.fromDegrees(0)));
                    startSnakeDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(4.453, 7.38, Rotation2d.fromDegrees(0)),
                                new Pose2d(7.714, 6.37, Rotation2d.fromDegrees(-83.199)),
                                new Pose2d(7.959, 1.337, Rotation2d.fromDegrees(62.354)),
                                new Pose2d(7.714, 7.121, Rotation2d.fromDegrees(152.765)),
                                new Pose2d(2.888, 7.121, Rotation2d.fromDegrees(-144.917)),
                                new Pose2d(1.102, 5.918, Rotation2d.fromDegrees(-174.806))),

                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-174.806)))

        );

        Command leftDepoOutpostLeftClimbAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(3.547, 6.021, Rotation2d.fromDegrees(180)));
                    startSnakeDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(3.547, 6.021, Rotation2d.fromDegrees(180)),
                                new Pose2d(1.480, 5.959, Rotation2d.fromDegrees(180))),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))),
                new WaitCommand(0.5),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(2.461, 5.516, Rotation2d.fromDegrees(-33.944)),
                                new Pose2d(1.852, 1.7, Rotation2d.fromDegrees(-138.652)),
                                new Pose2d(0.649, 0.729, Rotation2d.fromDegrees(-143.584)),
                                new Pose2d(2.629, 3.938, Rotation2d.fromDegrees(138.674))),
                        List.of(
                                new EventMarker("Stop Snake Drive", 1.07, new InstantCommand(() -> stopSnakeDrive())),
                                new EventMarker("Start Snake Drive", 1.69,
                                        new InstantCommand(() -> startSnakeDrive()))),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(-33.944)),
                        new GoalEndState(0, Rotation2d.fromDegrees(138.674))),
                new WaitCommand(2),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(2.629, 3.938, Rotation2d.fromDegrees(138.674)),
                                new Pose2d(1.089, 4.960, Rotation2d.fromDegrees(-150.593))),

                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(138.674)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-150.593))));

        Command leftMidPass = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(4.466, 7.367, Rotation2d.fromDegrees(0)));
                    startSnakeDrive();
                    startPassDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(4.466, 7.367, Rotation2d.fromDegrees(0)),
                                new Pose2d(7.636, 6.448, Rotation2d.fromDegrees(-66.501)),
                                new Pose2d(7.878, 0.696, Rotation2d.fromDegrees(-71.147)),
                                new Pose2d(7.878, 7.045, Rotation2d.fromDegrees(117.255)),
                                new Pose2d(7.476, 3.633, Rotation2d.fromDegrees(-91.146)),
                                new Pose2d(8.105, 0.994, Rotation2d.fromDegrees(70.602)),
                                new Pose2d(7.878, 7.045, Rotation2d.fromDegrees(95.464)),
                                new Pose2d(7.878, 0.696, Rotation2d.fromDegrees(-66.879)),
                                new Pose2d(7.054, 7.509, Rotation2d.fromDegrees(-163.009))),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(0)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-163.009))));

        Command leftDepoMidLeftClimb = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(3.547, 6.021, Rotation2d.fromDegrees(180)));
                    startPassDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(3.547, 6.021, Rotation2d.fromDegrees(180)),
                                new Pose2d(1.48, 5.959, Rotation2d.fromDegrees(180))),
                        List.of(),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))),
                new WaitCommand(0.5),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(2.879, 6.436, Rotation2d.fromDegrees(45.556)),
                                new Pose2d(4.353, 7.385, Rotation2d.fromDegrees(-2.694)),
                                new Pose2d(7.683, 6.436, Rotation2d.fromDegrees(-83.581)),
                                new Pose2d(7.806, 0.943, Rotation2d.fromDegrees(-69.928)),
                                new Pose2d(7.301, 7.003, Rotation2d.fromDegrees(157.574)),
                                new Pose2d(4.353, 7.488, Rotation2d.fromDegrees(-176.634)),
                                new Pose2d(1.076, 4.675, Rotation2d.fromDegrees(-107.966))),
                        List.of(
                                new EventMarker("Stop Snake Drive", 5.28)),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(45.556)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-107.966))));

        Command rightOutMidPassRightClimbAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(3.615, 0.626, Rotation2d.fromDegrees(180)));
                    startPassDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(3.615, 0.626, Rotation2d.fromDegrees(180)),
                                new Pose2d(0.719, 0.626, Rotation2d.fromDegrees(180))),
                        List.of(),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))),
                new WaitCommand(2),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(5.833, 0.779, Rotation2d.fromDegrees(18.187)),
                                new Pose2d(7.625, 1.740, Rotation2d.fromDegrees(79.051)),
                                new Pose2d(7.806, 7.117, Rotation2d.fromDegrees(-69.928)),
                                new Pose2d(7.806, 1.074, Rotation2d.fromDegrees(-135.294)),
                                new Pose2d(4.172, 0.626, Rotation2d.fromDegrees(-176.634)),
                                new Pose2d(1.101, 2.866, Rotation2d.fromDegrees(83.387))),
                        List.of(
                                new EventMarker("Stop Snake Drive", 5.28)),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(18.187)),
                        new GoalEndState(0, Rotation2d.fromDegrees(83.387))));

        Command testTurningAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(7.765, 5.943, Rotation2d.fromDegrees(-90)));
                    startSnakeDrive();
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(7.765, 5.943, Rotation2d.fromDegrees(-90)),
                                new Pose2d(7.878, 2.359, Rotation2d.fromDegrees(-71.147)),
                                new Pose2d(7.878, 6.125, Rotation2d.fromDegrees(-112.380))),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(-90)),
                        new GoalEndState(0, Rotation2d.fromDegrees(-112.380)))

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

        // Command rightOutMidAuto = new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // Drivetrain.getInstance().setStartingPose(new Pose2d(4.467, 0.669,
        // Rotation2d.fromDegrees(180)));
        // }),
        // new AutoDriveCommand(
        // List.of(
        // new Pose2d(4.467, FieldConstants.kFieldSize.getY() - 7.587,
        // Rotation2d.fromDegrees(-16.928)),
        // )
        // )
        // );

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

        Command rightOutDepClimbAuto = new SequentialCommandGroup(
                new InstantCommand(() -> {
                    Drivetrain.getInstance().setStartingPose(new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)));
                }),
                new AutoDriveCommand(
                        List.of(
                                new Pose2d(3.586, 0.639, Rotation2d.fromDegrees(180)),
                                new Pose2d(0.856, 0.639, Rotation2d.fromDegrees(180))),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(3.0, 4.0, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))),

                new WaitCommand(2),

                new AutoDriveCommand(
                        List.of(
                                new Pose2d(0.856, 0.639, Rotation2d.fromDegrees(180.000)),
                                new Pose2d(2.599, 3.434, Rotation2d.fromDegrees(71.696)),
                                new Pose2d(2.163, 5.918, Rotation2d.fromDegrees(180)),
                                new Pose2d(1.5, 5.918, Rotation2d.fromDegrees(0))),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(3.0, 4.0, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(180))),

                new WaitCommand(0.5),

                new AutoDriveCommand(
                        List.of(
                                new Pose2d(1.5, 5.918, Rotation2d.fromDegrees(0)),
                                new Pose2d(1.063, 4.833, Rotation2d.fromDegrees(-137.666))),
                        new PathConstraints(0.5, 0.5, 3 * Math.PI, 4 * Math.PI),
                        // new PathConstraints(3.0, 4.0, 3 * Math.PI, 4 * Math.PI),
                        new IdealStartingState(0, Rotation2d.fromDegrees(180)),
                        new GoalEndState(0, Rotation2d.fromDegrees(0)))

        );

        // Command outpostAuto = new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // Drivetrain.getInstance().setStartingPose(new Pose2d(3.599, 0.664,
        // Rotation2d.fromDegrees(-90)));
        // }),
        // new AutoDriveCommand(
        // List.of(
        // new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(0)),
        // new Pose2d(0.584,0.664, Rotation2d.fromDegrees(0))
        // ),
        // new PathConstraints(2, 4, Math.PI, Math.PI),
        // // new PathConstraints(3.0, 4.0, 3 * Math.PI, 4 * Math.PI),
        // new IdealStartingState(0, Rotation2d.fromDegrees(0)),
        // new GoalEndState(0, Rotation2d.fromDegrees(0))
        // )
        // );

        // Command notsosquiggleAuto = new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // Drivetrain.getInstance().setStartingPose(new Pose2d(3.599, 0.664,
        // Rotation2d.fromDegrees(180)));
        // startSnakeDrive();
        // }),
        // new AutoDriveCommand(
        // List.of(
        // new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(175.355)),
        // new Pose2d(2.202,2.735, Rotation2d.fromDegrees(100.909)),
        // new Pose2d(2.034,5.297, Rotation2d.fromDegrees(90))
        // ),
        // new PathConstraints(1, 1, 6 * Math.PI, 8 * Math.PI),
        // // new PathConstraints(3.0, 4.0, 3 * Math.PI, 4 * Math.PI),
        // new IdealStartingState(0, Rotation2d.fromDegrees(0)),
        // new GoalEndState(0, Rotation2d.fromDegrees(0))
        // )
        // );

        // Command squiggleAuto = new SequentialCommandGroup(
        // new InstantCommand(() -> {
        // Drivetrain.getInstance().setStartingPose(new Pose2d(3.599, 0.664,
        // Rotation2d.fromDegrees(180)));
        // startSnakeDrive();
        // }),
        // new AutoDriveCommand(
        // List.of(
        // new Pose2d(3.599, 0.664, Rotation2d.fromDegrees(149.577)),
        // new Pose2d(2.034,2.541, Rotation2d.fromDegrees(42.614)),
        // new Pose2d(2.771,3.912, Rotation2d.fromDegrees(81.251)),
        // new Pose2d(2.034,5.297, Rotation2d.fromDegrees(77.259))
        // ),
        // new PathConstraints(1, 1, 3 * Math.PI, 4* Math.PI),
        // // new PathConstraints(3.0, 4.0, 3 * Math.PI, 4 * Math.PI),
        // new IdealStartingState(0, Rotation2d.fromDegrees(0)),
        // new GoalEndState(0, Rotation2d.fromDegrees(0))
        // )
        // );

        autoChooser.addOption("1AutoCommand", autoCommand);
        autoChooser.addOption("R - Outpost/Deport/Left Climb Auto", rightOutDepClimbAuto);
        autoChooser.addOption("R - Basic Outpost Auto", basicRightOutpostAuto);
        autoChooser.addOption("R - Outpost/Depot/Left Climb Auto", rightOutDepClimbLAuto);
        autoChooser.addOption("R - Mid/Left Climb Auto", rightMidLeftClimbAuto);
        autoChooser.addOption("L - Mid/Store/Depot Auto", leftMidStoreDepotAuto);
        autoChooser.addOption("L - Depost/Outpost/Right Climb Auto", leftDepOutClimbRAuto);
        autoChooser.addOption("L - Mid/Outpost/Left Climb Auto", leftMidOutClimbRAuto);
        autoChooser.addOption("L - Depot/Outpost/Left Climb Auto", leftDepoOutpostLeftClimbAuto);
        autoChooser.addOption("L - Mid (Pass + Store)/Left Climb Auto", leftMidPassScoreClimbLAuto);
        autoChooser.addOption("L - Pass Only Loop", leftMidPass);
        autoChooser.addOption("L - Depot/Mid/Left Climb Auto", leftDepoMidLeftClimb);
        autoChooser.addOption("R - Outpost/ Mid (Pass) + Right Climb Auto", rightOutMidPassRightClimbAuto);
        autoChooser.addOption("Test Turning", testTurningAuto);
        autoChooser.addOption("Left Choate Trench Auto", choateAuto);
        autoChooser.addOption("R - Mid (Pass + Score)/Right Climb Auto", rightMidPassScoreClimbLAuto);
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