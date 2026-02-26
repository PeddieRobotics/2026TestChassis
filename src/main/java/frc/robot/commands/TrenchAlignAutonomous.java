package frc.robot.commands;

import java.util.Optional;

import javax.lang.model.util.ElementScanner14;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AddressableLED;
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

public class TrenchAlignAutonomous extends Command {
    private Drivetrain drivetrain;

    private Limelight llFront;
    private double
    // yTarget,
    rotTargetBlue;
    private Pose2d odometry;
    private boolean canPassTrench;

    private Translation2d target;
    private Translation2d endTarget;

    private PIDController xController, yController, rotController;

    private Translation2d inFarOffset = TrenchAlignConstants.kInFarOffset;
    private Translation2d inCloseOffset = TrenchAlignConstants.kInCloseOffset;
    private Translation2d outOffset = TrenchAlignConstants.kOutOffset;

    private double Px = TrenchAlignConstants.kPx;
    private double Ix = TrenchAlignConstants.kIx;
    private double Dx = TrenchAlignConstants.kDx;
    private double FFx = TrenchAlignConstants.kFFx;

    private double Py = TrenchAlignConstants.kPy;
    private double Iy = TrenchAlignConstants.kIy;
    private double Dy = TrenchAlignConstants.kDy;
    private double FFy = TrenchAlignConstants.kFFy;

    private double Pr = TrenchAlignConstants.kPr;
    private double Ir = TrenchAlignConstants.kIr;
    private double Dr = TrenchAlignConstants.kDr;
    private double FFr = TrenchAlignConstants.kFFr;

    private double stage1speed = TrenchAlignConstants.kStage1Speed;
    private double stage2speed = TrenchAlignConstants.kStage2Speed;
        
    private double yEpsilon = TrenchAlignConstants.kEpsilonY;
    private double rotEpsilon = TrenchAlignConstants.kEpsilonRot;

    public TrenchAlignAutonomous() {
        drivetrain = Drivetrain.getInstance();
        llFront = LimelightFront.getInstance();

        xController = new PIDController(TrenchAlignConstants.kPx, TrenchAlignConstants.kIx, TrenchAlignConstants.kDx);
        yController = new PIDController(TrenchAlignConstants.kPy, TrenchAlignConstants.kIy, TrenchAlignConstants.kDy);
        rotController = new PIDController(TrenchAlignConstants.kPr, TrenchAlignConstants.kIr, TrenchAlignConstants.kDr);
        rotController.enableContinuousInput(-180, 180);
    }

    @Override
    public void initialize() {
        addRequirements(drivetrain);
        inFarOffset = new Translation2d(inFarOffset.getX(), 0);        
        inCloseOffset = new Translation2d(inCloseOffset.getX(), 0);

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
        double absRotError = 0;

        // driving forward (plus direction)
        if (odometry.getX() < closestTrench.getX()) {
            Translation2d farTarget = closestTrench.minus(inFarOffset);
            endTarget = closestTrench.plus(outOffset);
            rotTargetBlue = 0;
            absRotError= Math.abs(Math.IEEEremainder(currentHeading - rotTargetBlue, 360));
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
            rotTargetBlue = 180;
            absRotError= Math.abs(Math.IEEEremainder(currentHeading - rotTargetBlue, 360));
            // "ahead of" the far target
            if (odometry.getX() > farTarget.getX() && absRotError < 40)
                target = closestTrench.plus(inCloseOffset);
            else
                target = farTarget;
        }

    }

    @Override
    public void execute() {
        xController.setPID(Px, Ix, Dx);
        yController.setPID(Py, Iy, Dy);
        rotController.setPID(Pr, Ir, Dr);

        Optional<Pose2d> llOdometry = llFront.getPoseMT2();
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
                new Translation2d(rotTargetBlue == 0 ? stage2speed : -stage2speed, yVel),
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
        return rotTargetBlue == 0 ? odometry.getX() > endTarget.getX() : odometry.getX() < endTarget.getX();
    }
}