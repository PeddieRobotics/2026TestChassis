package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.LimelightLeft;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LimelightRight;
import frc.robot.utils.ShooterUtil;
import frc.robot.utils.Constants.*;
import frc.robot.utils.ShooterUtil.ShootingParameters;

public class LockOnTurret extends Command {
    private final Turret turret;
    private final Drivetrain drivetrain;

    private Translation2d hub;

    public LockOnTurret() {
        SmartDashboard.putNumber("best pose x", 0);
        SmartDashboard.putNumber("best pose y", 0);
        SmartDashboard.putNumber("gyro heading", 0);
        SmartDashboard.putNumber("turret center x", 0);
        SmartDashboard.putNumber("turret center y", 0);
        SmartDashboard.putNumber("turret to hub x", 0);
        SmartDashboard.putNumber("turret to hub y", 0);
        SmartDashboard.putNumber("turret angle", 0);
        SmartDashboard.putNumber("target yaw", 0);
        SmartDashboard.putNumber("robot center x", 0);
        SmartDashboard.putNumber("robot center y", 0);
        SmartDashboard.putNumber("turret center x", 0);
        SmartDashboard.putNumber("turret center y", 0);
        SmartDashboard.putNumber("turret hub x", 0);
        SmartDashboard.putNumber("turret hub y", 0);
        drivetrain = Drivetrain.getInstance();
        turret = Turret.getInstance();

        addRequirements(turret);
    }

    @Override
    public void initialize() {
        hub = FieldConstants.getHub();
    }

    @Override
    public void execute() {
        final Translation2d robotCenter = drivetrain.getPose().getTranslation();

        final Translation2d turretCenter = robotCenter.plus(TurretConstants.kRobotCenterToTurretCenter.rotateBy(Rotation2d.fromDegrees(drivetrain.getHeadingBlue())));

        final Translation2d turretToHub = hub.minus(turretCenter);

        final ShootingParameters params = ShooterUtil.getShootingParameters(turretCenter, drivetrain.getCurrentTranslation(), turretToHub);

        turret.setAngleFieldRelative(params.yaw());
        // turret.setAngleFieldRelative(new Rotation2d(Math.atan2(turretToHub.getY(), turretToHub.getX())));

        // put translation2d's on dashboard to debug

        SmartDashboard.putNumber("robot center x", robotCenter.getX());
        SmartDashboard.putNumber("robot center y", robotCenter.getY());
        SmartDashboard.putNumber("turret center x", turretCenter.getX());
        SmartDashboard.putNumber("turret center y", turretCenter.getY());
        SmartDashboard.putNumber("turret hub x", turretToHub.getX());
        SmartDashboard.putNumber("turret hub y", turretToHub.getY());
        SmartDashboard.putNumber("turret hub angle", Math.atan2(turretToHub.getY(), turretToHub.getX()));
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
