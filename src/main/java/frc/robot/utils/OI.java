package frc.robot.utils;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.LockDrivetrain;
import frc.robot.commands.WheelRadiusCharacterization;
import frc.robot.subsystems.Drivetrain;
import frc.robot.utils.Constants.DriveConstants;

public class OI {
    private static OI oi;
    private PS4Controller controller;

    public OI() {
        controller = new PS4Controller(0);

        Trigger psButton = new JoystickButton(controller, PS4Controller.Button.kPS.value);
        psButton.onTrue(new InstantCommand(() -> Drivetrain.getInstance().setGyro(0)));

        Trigger R1Bumper = new JoystickButton(controller, PS4Controller.Button.kR1.value);
        R1Bumper.whileTrue(new LockDrivetrain());

        Trigger L1Bumper = new JoystickButton(controller, PS4Controller.Button.kL1.value);
        L1Bumper.whileTrue(new WheelRadiusCharacterization());

        SmartDashboard.putNumber("Drive: cardinal scale", 0.5);
    }

    public enum DPadDirection {
        NONE, FORWARDS, LEFT, RIGHT, BACKWARDS
    };

    public static OI getInstance(){
        if (oi == null)
            oi = new OI();
        return oi;
    }

    public double getForward() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public double getStrafe() {
        double val = -controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
        return Math.abs(val) < 0.1 ? 0 : val;
    }

    public Translation2d fromPolar(Rotation2d direction, double magnitude) {
        return new Translation2d(direction.getCos() * magnitude, direction.getSin() * magnitude);
    }

    public boolean leftTriggerHeld() {
        return controller.getL2Button();
    }

    public boolean rightTriggerHeld() {
        return controller.getR2Button();
    }

    public boolean anyTriggerHeld(){
        return leftTriggerHeld() || rightTriggerHeld();
    }

     public DPadDirection getDriverDPadInput() {
        switch (controller.getPOV()) {
            case 0:
                return DPadDirection.FORWARDS;
            case 90:
                return DPadDirection.RIGHT;
            case 270:
                return DPadDirection.LEFT;
            case 180:
                return DPadDirection.BACKWARDS;
            default:
                return DPadDirection.NONE;
        }
    }

    public Translation2d getSwerveTranslation() {
        double xSpeed = getForward();
        double ySpeed = getStrafe();

        double xSpeedCommanded;
        double ySpeedCommanded;

        xSpeedCommanded = xSpeed;
        ySpeedCommanded = ySpeed;

        Translation2d next_translation = new Translation2d(xSpeedCommanded, ySpeedCommanded);

        double norm = next_translation.getNorm();
        if (norm < 0.1) {
            return new Translation2d();
        } else {
            Rotation2d deadband_direction = new Rotation2d(next_translation.getX(), next_translation.getY());
            Translation2d deadband_vector = fromPolar(deadband_direction, 0.1);

            double new_translation_x = next_translation.getX()
                    - (deadband_vector.getX()) / (1 - deadband_vector.getX());
            double new_translation_y = next_translation.getY()
                    - (deadband_vector.getY()) / (1 - deadband_vector.getY());

            next_translation = new Translation2d(
                    new_translation_x * DriveConstants.kMaxFloorSpeed,
                    new_translation_y * DriveConstants.kMaxFloorSpeed);

            return next_translation;
        }
    }

    public double getRotation() {
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);

        double combinedRotation = (rightRotation - leftRotation) / 2.0;
        combinedRotation = Math.signum(combinedRotation) * Math.pow(combinedRotation, 2);

        return Math.abs(combinedRotation) < 0.05 ? 0 : combinedRotation * DriveConstants.kMaxRotationSpeed;
    }

    public Translation2d getCardinalDirection() {
        double cardinal = SmartDashboard.getNumber("Drive: cardinal scale", 0.0);
        switch (getDriverDPadInput()) {
            case FORWARDS:
                return new Translation2d(cardinal * DriveConstants.kMaxFloorSpeed,
                        0.0);
            case RIGHT:
                return new Translation2d(0.0,
                        -cardinal * DriveConstants.kMaxFloorSpeed);
            case LEFT:
                return new Translation2d(0.0,
                        cardinal * DriveConstants.kMaxFloorSpeed);
            case BACKWARDS:
                return new Translation2d(-cardinal * DriveConstants.kMaxFloorSpeed,
                        0.0);
            default:
                return new Translation2d(0.0, 0.0);
        }

    }

    public double getJoystickAngle(){
        Translation2d translation = getSwerveTranslation();
        return Units.radiansToDegrees(Math.atan2(translation.getY(), translation.getX()));
    }
}