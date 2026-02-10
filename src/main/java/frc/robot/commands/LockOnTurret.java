package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.subsystems.LimelightLeft;
import frc.robot.subsystems.Turret;
import frc.robot.subsystems.LimelightRight;
import frc.robot.utils.Constants.*;

public class LockOnTurret extends Command {
    private Limelight[] limelights;
    private Turret turret;
    private Drivetrain drivetrain;
    
    public LockOnTurret() {
        limelights = new Limelight[] {
            LimelightFront.getInstance(),
            LimelightBack.getInstance(),
            LimelightLeft.getInstance(),
            LimelightRight.getInstance()
        };

        drivetrain = Drivetrain.getInstance();
        
        turret = Turret.getInstance();
    }
    @Override
    public void initialize() {}

    @Override 
    public void execute() {
        // find closest limelight
        Optional<Pose2d> bestPose = Optional.empty();

        double bestTagCount = 0;
        double bestDistance = 10000;

        for (Limelight ll : limelights) {
            Optional<Pose2d> pose = ll.getPoseMT2();
            if (pose.isEmpty())
                continue;

            double tagCount = ll.getNumberOfTagsSeen();
            double distance = ll.getDistanceEstimatedPose();
            if (tagCount > bestTagCount || (tagCount == bestTagCount && distance < bestDistance)) {
                bestPose = pose;
                bestTagCount = tagCount;
                bestDistance = distance;
            }
        }

        if (bestPose.isEmpty())
            return;
        Translation2d robotCenter = bestPose.get().getTranslation(); // origin to robot center

        Translation2d turretCenter = robotCenter.rotateBy(Rotation2d.fromDegrees(drivetrain.getHeadingBlue())); // origin to turret center

        Translation2d hub = DriverStation.getAlliance().get()==DriverStation.Alliance.Blue ? 
                            new Translation2d(FieldConstants.blueHubPositionX,FieldConstants.blueHubPositionY) : 
                            new Translation2d(FieldConstants.redHubPositionX,FieldConstants.redHubPositionY);
        Translation2d turretHub = turretCenter.minus(hub);

        Rotation2d targetYaw = turretHub.getAngle();
        turret.setAngleFieldRelative(targetYaw);
    }
    
    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return false;
    }
    
}
