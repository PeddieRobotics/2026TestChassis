package frc.robot.commands;

import java.util.Optional;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.Constants.FieldConstants.TrenchLocations;
import frc.robot.utils.OI;

public class TrenchAlign extends Command {
    private OI oi;
    private Drivetrain drivetrain;
    private Limelight llFront;
    private double
    // yTarget,
    rotTargetBlue;
    private Pose2d odometry;
    private boolean canPassTrench;
    private boolean drivePlus;

    private Translation2d target;
    private Translation2d endTarget;

    private PIDController xController, yController, rotController;

    private static double yEpsilon = 0.4, rotEpsilon = 5;
    private static double finalMovementSpeed = 2.5;
    private static double kMaxSpeed = 2.5;
    private static final double kPx = 3.5, kIx = 0, kDx = 0, kFFx = 0;
    private static final double kPy = 3.5, kIy = 0, kDy = 0, kFFy = 0;
    private static final double kPr = 0.04, kIr = 0, kDr = 0, kFFr = 0;

    public enum TrenchOption {
        LEFT, RIGHT
    };

    private static Translation2d offset = new Translation2d(1.2, 0);

    public TrenchAlign(TrenchOption option) {
        drivetrain = Drivetrain.getInstance();
        llFront = LimelightFront.getInstance();

        xController = new PIDController(kPx, kIx, kDx);
        yController = new PIDController(kPy, kIy, kDy);
        rotController = new PIDController(kPr, kIr, kDr);
        rotController.enableContinuousInput(-180, 180);

        SmartDashboard.putNumber("TrenchAlign yEpsilon", yEpsilon);
        SmartDashboard.putNumber("TrenchAlign rotEpsilon", rotEpsilon);

        SmartDashboard.putNumber("TrenchAlign Px", kPx);
        SmartDashboard.putNumber("TrenchAlign Ix", kIx);
        SmartDashboard.putNumber("TrenchAlign Dx", kDx);
        SmartDashboard.putNumber("TrenchAlign FFx", kFFx);

        SmartDashboard.putNumber("TrenchAlign Py", kPy);
        SmartDashboard.putNumber("TrenchAlign Iy", kIy);
        SmartDashboard.putNumber("TrenchAlign Dy", kDy);
        SmartDashboard.putNumber("TrenchAlign FFy", kFFy);

        SmartDashboard.putNumber("TrenchAlign Pr", kPr);
        SmartDashboard.putNumber("TrenchAlign Ir", kIr);
        SmartDashboard.putNumber("TrenchAlign Dr", kDr);
        SmartDashboard.putNumber("TrenchAlign FFr", kFFr);

        SmartDashboard.putNumber("TrenchAlign offset", offset.getX());
        SmartDashboard.putNumber("TrenchAlign max speed", kMaxSpeed);
        SmartDashboard.putNumber("Robot Movement Speed", finalMovementSpeed);
    }

    @Override
    public void initialize() {
        oi = OI.getInstance();
        canPassTrench = false;

        // get closest trench
        odometry = drivetrain.getPose();

        Translation2d closestTrench = TrenchLocations.allTrenches[0];
        double closestDistance = 99999999;
        for (Translation2d trench : TrenchLocations.allTrenches) {
            double distance = odometry.getTranslation().getSquaredDistance(trench);
            if (distance < closestDistance) {
                closestTrench = trench;
                closestDistance = distance;
            }
        }

        offset = new Translation2d(SmartDashboard.getNumber("TrenchAlign offset", 0), 0);

        if (odometry.getX() < closestTrench.getX()) {
            target = closestTrench.minus(offset);
            endTarget = closestTrench.plus(offset);
            drivePlus = true;
        } else {
            target = closestTrench.plus(offset);
            endTarget = closestTrench.minus(offset);
            drivePlus = false;
        }

        double currentHeading = drivetrain.getHeadingBlue();
        rotTargetBlue = Math.abs(currentHeading) < 90 ? 0 : 180;
    }

    @Override
    public void execute() {
        double Px = SmartDashboard.getNumber("TrenchAlign Px", kPx);
        double Ix = SmartDashboard.getNumber("TrenchAlign Ix", kIx);
        double Dx = SmartDashboard.getNumber("TrenchAlign Dx", kDx);
        double FFx = SmartDashboard.getNumber("TrenchAlign FFx", kFFx);

        double Py = SmartDashboard.getNumber("TrenchAlign Py", kPy);
        double Iy = SmartDashboard.getNumber("TrenchAlign Iy", kIy);
        double Dy = SmartDashboard.getNumber("TrenchAlign Dy", kDy);
        double FFy = SmartDashboard.getNumber("TrenchAlign FFy", kFFy);

        double Pr = SmartDashboard.getNumber("TrenchAlign Pr", kPr);
        double Ir = SmartDashboard.getNumber("TrenchAlign Ir", kIr);
        double Dr = SmartDashboard.getNumber("TrenchAlign Dr", kDr);
        double FFr = SmartDashboard.getNumber("TrenchAlign FFr", kFFr);

        yEpsilon = SmartDashboard.getNumber("TrenchAlign yEpsilon", 0);
        rotEpsilon = SmartDashboard.getNumber("TrenchAlign rotEpsilon", rotEpsilon);
        kMaxSpeed = SmartDashboard.getNumber("TrenchAlign max speed", kMaxSpeed);
        double movementSpeed = SmartDashboard.getNumber("Robot Movement Speed", 0);

        xController.setPID(Px, Ix, Dx);
        yController.setPID(Py, Iy, Dy);
        rotController.setPID(Pr, Ir, Dr);

        Optional<Pose2d> odoOptional = llFront.getPoseMT2();
        odometry = odoOptional.isPresent() ? odoOptional.get() : drivetrain.getPose();

        double rotError = drivetrain.getHeadingBlue() - rotTargetBlue;
        double rotVel = rotController.calculate(rotError) - Math.signum(rotError) * FFr;

        double xError = odometry.getX() - target.getX();
        double xVel = xController.calculate(xError) - Math.signum(xError) * FFx;

        double yError = odometry.getY() - target.getY();
        double yVel = yController.calculate(yError) - Math.signum(yError) * FFy;

        Translation2d vel = new Translation2d(xVel, yVel);
        if (vel.getSquaredNorm() > kMaxSpeed * kMaxSpeed)
            vel = vel.div(vel.getNorm()).times(kMaxSpeed);

        if (Math.abs(yError) < yEpsilon && Math.abs(Math.IEEEremainder(rotError, 360)) < rotEpsilon)
            canPassTrench = true;

        if (!canPassTrench)
            drivetrain.driveBlue(vel, rotVel, true, new Translation2d(0, 0));
        else {
            drivetrain.driveBlue(
                    new Translation2d(drivePlus ? movementSpeed : -movementSpeed, yVel),
                    rotVel, true, new Translation2d(0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        // if (odometry.equals(new Pose2d()))
        // return false;
        return drivePlus ? odometry.getX() > endTarget.getX() : odometry.getX() < endTarget.getX();
    }
}