package frc.robot.commands;

import java.util.Optional;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.LimelightFront;
import frc.robot.utils.OI;

public class TrenchAlign extends Command {
    private OI oi;
    private Drivetrain drivetrain;
    private Limelight llFront;
    private double yTarget, rotTarget;
    
    private PIDController yController, rotController;
    
    private static final double kMaxSpeed = 2.0;
    private static final double kPy = 2.0, kIy = 0, kDy = 0, kFFy = 0;
    private static final double kPr = 0.04, kIr = 0, kDr = 0, kFFr = 0;

    public enum TrenchOption {
        LEFT, RIGHT
    };

    public TrenchAlign(TrenchOption option) {
        drivetrain = Drivetrain.getInstance();
        llFront = LimelightFront.getInstance();

        Alliance alliance = DriverStation.getAlliance().isEmpty() ? DriverStation.getAlliance().get() : Alliance.Blue;

        // right trench
        if ((alliance == Alliance.Blue && option == TrenchOption.RIGHT) || (alliance == Alliance.Red && option == TrenchOption.LEFT)) 
            yTarget = Units.inchesToMeters(50.59 - 24.97);
        // left trench
        else if ((alliance == Alliance.Blue && option == TrenchOption.LEFT) || (alliance == Alliance.Red && option == TrenchOption.RIGHT))
            yTarget = 8.07 - Units.inchesToMeters(50.59 - 24.97);

        // depends on side
        rotTarget = 0;
        
        yController = new PIDController(kPy, kIy, kDy);
        rotController = new PIDController(kPr, kIr, kDr);
        
        SmartDashboard.putNumber("TrenchPass Py", kPy);
        SmartDashboard.putNumber("TrenchPass Iy", kIy);
        SmartDashboard.putNumber("TrenchPass Dy", kDy);
        SmartDashboard.putNumber("TrenchPass FFy", kFFy);

        SmartDashboard.putNumber("TrenchPass Pr", kPr);
        SmartDashboard.putNumber("TrenchPass Ir", kIr);
        SmartDashboard.putNumber("TrenchPass Dr", kDr);
        SmartDashboard.putNumber("TrenchPass FFr", kFFr);
    }
    
    @Override
    public void initialize() {
        // TODO: MOVE HOOD DOWN
        oi = OI.getInstance();
    }

    @Override
    public void execute() {
        double Py = SmartDashboard.getNumber("TrenchPass Py", kPy);
        double Iy = SmartDashboard.getNumber("TrenchPass Iy", kIy);
        double Dy = SmartDashboard.getNumber("TrenchPass Dy", kDy);
        double FFy = SmartDashboard.getNumber("TrenchPass FFy", kFFy);

        double Pr = SmartDashboard.getNumber("TrenchPass Pr", kPr);
        double Ir = SmartDashboard.getNumber("TrenchPass Ir", kIr);
        double Dr = SmartDashboard.getNumber("TrenchPass Dr", kDr);
        double FFr = SmartDashboard.getNumber("TrenchPass FFr", kFFr);
        
        yController.setPID(Py, Iy, Dy);
        rotController.setPID(Pr, Ir, Dr);
        
        Optional<Pose2d> odoOptional = llFront.getPoseMT2();
        if (odoOptional.isEmpty())
            odoOptional = Optional.of(drivetrain.getPose());

        double rotError = drivetrain.getHeading() - rotTarget;
        double rotVel = rotController.calculate(rotError) - Math.signum(rotError) * FFr;
        
        double yError = odoOptional.get().getY() - yTarget;
        double yVel = yController.calculate(yError) - Math.signum(yError) * FFy;
        yVel = Math.min(Math.abs(yVel), kMaxSpeed) * Math.signum(yVel);
        
        // |(V_x, V_y)| = sqrt(V_x^2 + V_y^2) = V_max
        // double xVel = Math.sqrt(kMaxSpeed * kMaxSpeed - yVel * yVel);
        double xVel = oi.getSwerveTranslation().getX();

        Translation2d vel = new Translation2d(xVel, yVel);
        drivetrain.drive(vel, rotVel, true, new Translation2d(0, 0));
    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
        return false;
    }
}