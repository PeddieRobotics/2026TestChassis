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
import frc.robot.subsystems.LimelightBack;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.Constants.TrenchAlignConstants;
import frc.robot.utils.Constants.FieldConstants.TrenchLocations;
import frc.robot.utils.OI;

public class TrenchAlign extends Command {
    private OI oi;
    private Drivetrain drivetrain;

    private Limelight llFront, llBack;
    private Limelight theLimelight;

    private double
    // yTarget,
    rotTargetBlue;
    private Pose2d odometry;
    private boolean canPassTrench;
    private boolean drivePlus;

    private Translation2d target;
    private Translation2d endTarget;

    private PIDController xController, yController, rotController;

    private Translation2d inFarOffset = TrenchAlignConstants.kInFarOffset;
    private Translation2d inCloseOffset = TrenchAlignConstants.kInCloseOffset;
    private Translation2d outOffset = TrenchAlignConstants.kOutOffset;

    public enum TrenchOption {
        LEFT, RIGHT
    };

    public TrenchAlign(boolean isAuto) {
        drivetrain = Drivetrain.getInstance();
        llFront = LimelightFront.getInstance();
        llBack = LimelightBack.getInstance();

        xController = new PIDController(TrenchAlignConstants.kPx, TrenchAlignConstants.kIx, TrenchAlignConstants.kDx);
        yController = new PIDController(TrenchAlignConstants.kPy, TrenchAlignConstants.kIy, TrenchAlignConstants.kDy);
        rotController = new PIDController(TrenchAlignConstants.kPr, TrenchAlignConstants.kIr, TrenchAlignConstants.kDr);
        rotController.enableContinuousInput(-180, 180);

        if (isAuto){
            outOffset = TrenchLocations.kAutoOffset;
        }

        SmartDashboard.putNumber("TrenchAlign yEpsilon", TrenchAlignConstants.kEpsilonY);
        SmartDashboard.putNumber("TrenchAlign rotEpsilon", TrenchAlignConstants.kEpsilonRot);

        SmartDashboard.putNumber("TrenchAlign Px", TrenchAlignConstants.kPx);
        SmartDashboard.putNumber("TrenchAlign Ix", TrenchAlignConstants.kIx);
        SmartDashboard.putNumber("TrenchAlign Dx", TrenchAlignConstants.kDx);
        SmartDashboard.putNumber("TrenchAlign FFx", TrenchAlignConstants.kFFx);

        SmartDashboard.putNumber("TrenchAlign Py", TrenchAlignConstants.kPy);
        SmartDashboard.putNumber("TrenchAlign Iy", TrenchAlignConstants.kIy);
        SmartDashboard.putNumber("TrenchAlign Dy", TrenchAlignConstants.kDy);
        SmartDashboard.putNumber("TrenchAlign FFy", TrenchAlignConstants.kFFy);

        SmartDashboard.putNumber("TrenchAlign Pr", TrenchAlignConstants.kPr);
        SmartDashboard.putNumber("TrenchAlign Ir", TrenchAlignConstants.kIr);
        SmartDashboard.putNumber("TrenchAlign Dr", TrenchAlignConstants.kDr);
        SmartDashboard.putNumber("TrenchAlign FFr", TrenchAlignConstants.kFFr);

        SmartDashboard.putNumber("TrenchAlign offset", inFarOffset.getX());
        SmartDashboard.putNumber("TrenchAlign closeOffset", inCloseOffset.getX());
        SmartDashboard.putNumber("TrenchAlign speed stage 1", TrenchAlignConstants.kStage1Speed);
        SmartDashboard.putNumber("TrenchAlign speed stage 2", TrenchAlignConstants.kStage2Speed);
    }

    @Override
    public void initialize() {
        // inFarOffset = new Translation2d(SmartDashboard.getNumber("TrenchAlign offset", 0), 0);        
        // inCloseOffset = new Translation2d(SmartDashboard.getNumber("TrenchAlign closeOffset", 0), 0);

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

        double currentHeading = drivetrain.getHeadingBlue();
        rotTargetBlue = Math.abs(currentHeading) < 90 ? 0 : 180;
        double absRotError = Math.abs(Math.IEEEremainder(currentHeading - rotTargetBlue, 360));

        boolean blue = DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == Alliance.Blue;
        double rotTarget = blue ? rotTargetBlue : (rotTargetBlue + 180) % 360;

        // driving forward (plus direction)
        if (odometry.getX() < closestTrench.getX()) {
            Translation2d farTarget = closestTrench.minus(inFarOffset);
            endTarget = closestTrench.plus(outOffset);
            drivePlus = true;

            theLimelight = rotTarget == 0 ? llFront : llBack;

            // "behind" the far target
            if (odometry.getX() < farTarget.getX() && absRotError < 40)
                target = closestTrench.minus(inCloseOffset);
            else
                target = farTarget;
        }
        // driving backward (minus direction)
        else {
            Translation2d farTarget = closestTrench.plus(inFarOffset);
            endTarget = closestTrench.minus(outOffset);
            drivePlus = false;

            theLimelight = rotTarget == 0 ? llBack : llFront;

            // "ahead of" the far target
            if (odometry.getX() > farTarget.getX() && absRotError < 40)
                target = closestTrench.plus(inCloseOffset);
            else
                target = farTarget;
        }
    }

    @Override
    public void execute() {
        double Px = SmartDashboard.getNumber("TrenchAlign Px", 0);
        double Ix = SmartDashboard.getNumber("TrenchAlign Ix", 0);
        double Dx = SmartDashboard.getNumber("TrenchAlign Dx", 0);
        double FFx = SmartDashboard.getNumber("TrenchAlign FFx", 0);

        double Py = SmartDashboard.getNumber("TrenchAlign Py", 0);
        double Iy = SmartDashboard.getNumber("TrenchAlign Iy", 0);
        double Dy = SmartDashboard.getNumber("TrenchAlign Dy", 0);
        double FFy = SmartDashboard.getNumber("TrenchAlign FFy", 0);

        double Pr = SmartDashboard.getNumber("TrenchAlign Pr", 0);
        double Ir = SmartDashboard.getNumber("TrenchAlign Ir", 0);
        double Dr = SmartDashboard.getNumber("TrenchAlign Dr", 0);
        double FFr = SmartDashboard.getNumber("TrenchAlign FFr", 0);

        double stage1speed = SmartDashboard.getNumber("TrenchAlign speed stage 1", 0);
        double stage2speed = SmartDashboard.getNumber("TrenchAlign speed stage 2", 0);
        
        double yEpsilon = SmartDashboard.getNumber("TrenchAlign yEpsilon", 0);
        double rotEpsilon = SmartDashboard.getNumber("TrenchAlign rotEpsilon", 0);

        xController.setPID(Px, Ix, Dx);
        yController.setPID(Py, Iy, Dy);
        rotController.setPID(Pr, Ir, Dr);

        Optional<Pose2d> llOdometry = theLimelight.getPoseMT2();
        odometry = llOdometry.isPresent() ? llOdometry.get() : drivetrain.getPose();

        double rotError = drivetrain.getHeadingBlue() - rotTargetBlue;
        double rotVel = rotController.calculate(rotError) - Math.signum(rotError) * FFr;

        double xError = odometry.getX() - target.getX();
        double xVel = xController.calculate(xError) - Math.signum(xError) * FFx;

        double yError = odometry.getY() - target.getY();
        double yVel = yController.calculate(yError) - Math.signum(yError) * FFy;

        Translation2d vel = new Translation2d(xVel, yVel);
        if (vel.getSquaredNorm() > stage1speed * stage1speed)
            vel = vel.div(vel.getNorm()).times(stage1speed);

        if (Math.abs(yError) < yEpsilon && Math.abs(Math.IEEEremainder(rotError, 360)) < rotEpsilon)
            canPassTrench = true;

        if (!canPassTrench)
            drivetrain.driveBlue(vel, rotVel, true, new Translation2d(0, 0));
        else {
            drivetrain.driveBlue(
                new Translation2d(drivePlus ? stage2speed : -stage2speed, yVel),
                rotVel, true, new Translation2d(0, 0)
            );
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