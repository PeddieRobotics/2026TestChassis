package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.autos.MidOutpostAuto;
import frc.robot.autos.RightChoateAuto;
import frc.robot.autos.RightMidLeftClimbAuto;
import frc.robot.autos.TrenchTestAuto;
import frc.robot.subsystems.Drivetrain;

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

    public static void startSnakeDrive() {
        PPHolonomicDriveController.overrideRotationFeedback(() -> {
            return Drivetrain.getInstance().getRotationOverride();
        });
    }

    public static void stopSnakeDrive() {
        PPHolonomicDriveController.clearRotationFeedbackOverride();
    }

    public static void startPassDrive() {
        // TODO!!!! implement passDrive
    }

    public static void stopPassDrive() {
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

        autoChooser.addOption("R - Mid/Left Climb Auto", RightMidLeftClimbAuto.auto);
        autoChooser.addOption("Right Choate Trench Auto", RightChoateAuto.auto);
        autoChooser.addOption("Left Mid Outpost Auto", MidOutpostAuto.auto);
        autoChooser.addOption("Left Trench Test Auto", TrenchTestAuto.auto);

        SmartDashboard.putData("Auto Routines", autoChooser);

        autoStartPosition = new SendableChooser<>();
        autoStartPosition.setDefaultOption("1anywhere", 0.0);

        SmartDashboard.putData("Auto Starting Position", autoStartPosition);
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}
