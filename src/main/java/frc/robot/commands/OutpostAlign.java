package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.OutpostAlignConstants;

public class OutpostAlign extends Command {
     private Drivetrain drivetrain;
     private Limelight llFront;
     private Pose2d odometry;
     private Pose2d tagPose;
     private double Px, Ix, Dx, FFx, Py, Iy, Dy, Pr, FFy, LowerPr, Ir, Dr, FFr, rotLowerPThreshold;
     private double rotEpsilon, xEpsilon, yEpsilon;
     private PIDController xController, yController, rotController;
     private Translation2d target;
     private int desiredTag;
     private int desiredAngle;

     public OutpostAlign() {
          drivetrain = Drivetrain.getInstance();
          llFront = LimelightFront.getInstance();
          odometry = new Pose2d();

          Px = OutpostAlignConstants.kPx;
          Ix = OutpostAlignConstants.kIx;
          Dx = OutpostAlignConstants.kDx;
          FFx = OutpostAlignConstants.kFFx;
          xController = new PIDController(Px, Dx, Ix);

          Py = OutpostAlignConstants.kPy;
          Iy = OutpostAlignConstants.kIy;
          Dy = OutpostAlignConstants.kDy;
          FFy = OutpostAlignConstants.kFFy;
          yController = new PIDController(Py, Iy, Dy);

          Pr = OutpostAlignConstants.kPr;
          LowerPr = OutpostAlignConstants.kLowerPr;
          Ir = OutpostAlignConstants.kIr;
          Dr = OutpostAlignConstants.kDr;
          FFr = OutpostAlignConstants.kFFr;
          rotController = new PIDController(Pr, Ir, Dr);
          rotController.enableContinuousInput(-180, 180);

          xEpsilon = OutpostAlignConstants.kXEpsilon;
          yEpsilon = OutpostAlignConstants.kYEpsilon;
          rotEpsilon = OutpostAlignConstants.kRotEpsilon;
          rotLowerPThreshold = OutpostAlignConstants.kRLowerPThreshold;

          // SmartDashboard.putNumber("OutpostAlign Rot Lower P Threshold",
          // OutpostAlignConstants.kRLowerPThreshold);

          // SmartDashboard.putNumber("OutpostAlign xEpsilon",
          // OutpostAlignConstants.kXEpsilon);
          // SmartDashboard.putNumber("OutpostAlign yEpsilon",
          // OutpostAlignConstants.kYEpsilon);
          // SmartDashboard.putNumber("OutpostAlign rotEpsilon",
          // OutpostAlignConstants.kRotEpsilon);

          // SmartDashboard.putNumber("OutpostAlign Px", OutpostAlignConstants.kPx);
          // SmartDashboard.putNumber("OutpostAlign Ix", OutpostAlignConstants.kIx);
          // SmartDashboard.putNumber("OutpostAlign Dx", OutpostAlignConstants.kDx);
          // SmartDashboard.putNumber("OutpostAlign FFx", OutpostAlignConstants.kFFx);

          // SmartDashboard.putNumber("OutpostAlign Py", OutpostAlignConstants.kPy);
          // SmartDashboard.putNumber("OutpostAlign Iy", OutpostAlignConstants.kIy);
          // SmartDashboard.putNumber("OutpostAlign Dy", OutpostAlignConstants.kDy);
          // SmartDashboard.putNumber("OutpostAlign FFy", OutpostAlignConstants.kFFy);

          // SmartDashboard.putNumber("OutpostAlign Pr", OutpostAlignConstants.kPr);
          // SmartDashboard.putNumber("OutpostAlign Ir", OutpostAlignConstants.kIr);
          // SmartDashboard.putNumber("OutpostAlign Dr", OutpostAlignConstants.kDr);
          // SmartDashboard.putNumber("OutpostAlign LowerPr",
          // OutpostAlignConstants.kLowerPr);
          // SmartDashboard.putNumber("OutpostAlign FFr", OutpostAlignConstants.kFFr);

     }

     public boolean isInBlueSide() {
          return drivetrain.getPose().getX() < FieldConstants.kFieldSize.getX() / 2;
     }

     @Override
     public void initialize() {
          desiredTag = isInBlueSide() ? 29 : 13;
          desiredAngle = isInBlueSide() ? 180 : 0;
          tagPose = Limelight.getAprilTagPose(desiredTag);
          target = tagPose.getTranslation();
          target = isInBlueSide() ? target.plus(OutpostAlignConstants.kOutpostOffset)
                    : target.minus(OutpostAlignConstants.kOutpostOffset);

     }

     @Override
     public void execute() {
          // double Px = SmartDashboard.getNumber("OutpostAlign Px",
          // OutpostAlignConstants.kPx);
          // double Ix = SmartDashboard.getNumber("OutpostAlign Ix",
          // OutpostAlignConstants.kIx);
          // double Dx = SmartDashboard.getNumber("OutpostAlign Dx",
          // OutpostAlignConstants.kDx);
          // double FFx = SmartDashboard.getNumber("OutpostAlign FFx",
          // OutpostAlignConstants.kFFx);

          // double Py = SmartDashboard.getNumber("OutpostAlign Py",
          // OutpostAlignConstants.kPy);
          // double Iy = SmartDashboard.getNumber("OutpostAlign Iy",
          // OutpostAlignConstants.kIy);
          // double Dy = SmartDashboard.getNumber("OutpostAlign Dy",
          // OutpostAlignConstants.kDy);
          // double FFy = SmartDashboard.getNumber("OutpostAlign FFy",
          // OutpostAlignConstants.kFFy);

          // double Pr = SmartDashboard.getNumber("OutpostAlign Pr",
          // OutpostAlignConstants.kPy);
          // double LowerPr = SmartDashboard.getNumber("OutpostAlign LowerPr",
          // OutpostAlignConstants.kLowerPr);
          // double Ir = SmartDashboard.getNumber("OutpostAlign Ir",
          // OutpostAlignConstants.kIy);
          // double Dr = SmartDashboard.getNumber("OutpostAlign Dr",
          // OutpostAlignConstants.kDy);
          // double FFr = SmartDashboard.getNumber("OutpostAlign FFr",
          // OutpostAlignConstants.kFFy);

          // double xEpsilon = SmartDashboard.getNumber("OutpostAlign xEpsilon",
          // OutpostAlignConstants.kXEpsilon);
          // double yEpsilon = SmartDashboard.getNumber("OutpostAlign yEpsilon",
          // OutpostAlignConstants.kXEpsilon);
          // double rotEpsilon = SmartDashboard.getNumber("OutpostAlign rotEpsilon",
          // OutpostAlignConstants.kRotEpsilon);
          // double rotLowerPThreshold = SmartDashboard.getNumber("OutpostAlign Rot Lower
          // P Threshold", OutpostAlignConstants.kRLowerPThreshold);


          Optional<Pose2d> llOdometry = llFront.getPoseMT2();
          odometry = llOdometry.isPresent() ? llOdometry.get() : drivetrain.getPose();
          double rotError = drivetrain.getHeadingBlue() - desiredAngle;
          double xError = odometry.getX() - target.getX();
          double yError = odometry.getY() - target.getY();
          if (Math.abs(rotError) < rotLowerPThreshold)
               rotController.setP(LowerPr);

          System.out.println("ROTATIONAL ERROR" + rotError);
          double rotVel = 0;
          if (Math.abs(rotError) > rotEpsilon)
               rotVel = rotController.calculate(rotError) + Math.signum(rotError) * FFr;
          System.out.println("ROTATIONAL VEL"+rotVel);
          double xVel = 0;
          if (Math.abs(xError) > xEpsilon)
               xVel = xController.calculate(xError) + Math.signum(xError) * FFx;

          double yVel = 0;
          if (Math.abs(yError) > yEpsilon)
               yVel = yController.calculate(yError) + Math.signum(yError) * FFy;

          drivetrain.drive(new Translation2d(xVel, yVel), rotVel, true, null);
     }

     @Override
     public void end(boolean interrupted) {
     }

     @Override
     public boolean isFinished() {
          return false;
     }
}
