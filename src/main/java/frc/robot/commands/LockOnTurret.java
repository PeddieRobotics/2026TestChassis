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
    private final Limelight[] limelights;
    private final Turret turret;
    private final Drivetrain drivetrain;
    
    private final Translation2d hub;
    private final boolean teamBlue;
    
    public LockOnTurret() {
        limelights = new Limelight[] {
            LimelightFront.getInstance(),
            LimelightLeft.getInstance(),
            LimelightBack.getInstance(),
            LimelightRight.getInstance()
        };

        drivetrain = Drivetrain.getInstance();
        turret = Turret.getInstance();
        
        hub = FieldConstants.getHub();
        teamBlue = hub.equals(FieldConstants.kBlueHub);
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
            
            int lastTag = ll.getLastSeenTag();
            boolean isHub = (teamBlue && lastTag >= 18 && lastTag <= 27) || (!teamBlue && lastTag >= 2 && lastTag <= 11);
            
            if (isHub && tagCount > bestTagCount || (tagCount == bestTagCount && distance < bestDistance)) {
                bestPose = pose;
                bestTagCount = tagCount;
                bestDistance = distance;
            }
        }

        if (bestPose.isEmpty())
            return;

        final Translation2d robotCenter = bestPose.get().getTranslation(); // origin to robot center

        final Translation2d turretCenter = robotCenter.rotateBy(Rotation2d.fromDegrees(drivetrain.getHeadingBlue())); // origin to turret center

        final Translation2d turretHub = turretCenter.minus(hub);

        final Rotation2d targetYaw = turretHub.getAngle();

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
