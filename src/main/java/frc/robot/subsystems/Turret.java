package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.FieldConstants;
import frc.robot.utils.Constants.TurretConstants;
import frc.robot.commands.LockOnTurret;
import frc.robot.utils.Kraken;
// import frc.robot.utils.LimelightHelpers;
// import frc.robot.utils.OI;

public class Turret extends SubsystemBase {
    private static Turret turret;

    private Drivetrain drivetrain;

    private Limelight[] limelights; 

    // private OI oi;
    // private Limelight llTurret;

    private Kraken turretMotor;
    private CANcoder encoder1, encoder2;
    
    // rotation adjustment feedforward
    private double kRs = 0.05;
    private double kRv = 0.20;

    private double targetAngle;
    
    public Turret() {
        SmartDashboard.putNumber("heading", 0);
        // oi = OI.getInstance();
        // llTurret = LimelightTurret.getInstance();

        limelights = new Limelight[4];
        limelights[0] = LimelightBack.getInstance();
        limelights[1] = LimelightFront.getInstance();
        limelights[2] = LimelightLeft.getInstance();
        limelights[3] = LimelightRight.getInstance();

        CANBus canbus = new CANBus();

        encoder1 = new CANcoder(TurretConstants.kEncoderId1);
        encoder2 = new CANcoder(TurretConstants.kEncoderId2);
        
        double startingPositionTeethRaw = TurretConstants.kZeroPositionTeethRaw;

        CANcoderConfiguration config1 = new CANcoderConfiguration();
        // Setting this to 1 makes the absolute position
        // unsigned [0, 1)
        config1.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config1.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;

        if (TurretConstants.ZEROING_MODE)
            config1.MagnetSensor.MagnetOffset = 0;
        else {
            config1.MagnetSensor.MagnetOffset = TurretConstants.kEncoder1MagnetOffset;
            config1.MagnetSensor.MagnetOffset += (1.0 * startingPositionTeethRaw % TurretConstants.kEncoderGear1Teeth) / TurretConstants.kEncoderGear1Teeth;
        }

        encoder1.getConfigurator().apply(config1); 

        CANcoderConfiguration config2 = new CANcoderConfiguration();
        config2.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 1;
        config2.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        
        if (TurretConstants.ZEROING_MODE)
            config2.MagnetSensor.MagnetOffset = 0;
        else {
            config2.MagnetSensor.MagnetOffset = TurretConstants.kEncoder2MagnetOffset;
            config2.MagnetSensor.MagnetOffset += (1.0 * startingPositionTeethRaw % TurretConstants.kEncoderGear2Teeth) / TurretConstants.kEncoderGear2Teeth;
        }

        encoder2.getConfigurator().apply(config2); 

        turretMotor = new Kraken(TurretConstants.kTurretMotorDeviceId, canbus);
        turretMotor.setStatorCurrentLimit(40);
        turretMotor.setSupplyCurrentLimit(40);
        // inverted = CW+, gear inverses direction -> CCW+ for turret

        turretMotor.setSensorToMechanismRatio(TurretConstants.kKrakenToTurretRatio);
        turretMotor.setSoftLimits(true, TurretConstants.kMaxPositionRotations, TurretConstants.kMinPositionRotations);
        turretMotor.setInverted(true);
        turretMotor.setPIDValues(TurretConstants.kTurretYawP, TurretConstants.kTurretYawI, TurretConstants.kTurretYawD);
        turretMotor.setMotionMagicParameters(TurretConstants.kCruiseVelocity,TurretConstants.kMaxAcceleration,0);//no jerk

        turretMotor.setCoast();
        
        setMotorEncoder();

        targetAngle = 0.0;

        SmartDashboard.putNumber("Turret kRs", kRs);
        SmartDashboard.putNumber("Turret kRv", kRv);
        SmartDashboard.putNumber("Turret target angle", 0);

        drivetrain = Drivetrain.getInstance();
    }
    
    public static Turret getInstance() {
        if (turret == null)
            turret = new Turret();
        return turret;
    }
    
    public void setMotorEncoder() {
        turretMotor.setEncoder((getCurrentPositionTeethRaw() - TurretConstants.kZeroPositionTeethRaw) / TurretConstants.kTurretGearTeeth);
    }
    
    // this returns a value between 0 and n_1 * n_2
    // represents the number of teeth of the turret gear
    private double getCurrentPositionTeethRaw() {
        // n_1 teeth in gear 1, n_2 teeth in gear 2
        // a, b are position of gears 1, 2 have turned (number of fears)
        // 0 <= a < 20, 0 <= b < 21
        // call c_1, c_2 the reading of the CANcoders: these are in [0, 1)
        // so a = c_1 * n_1, b = c_2 * n_2
        // 
        // also remember that m_1 = n_2, m_2 = n_1
        //
        // K ≡ a (mod n_1)
        // K ≡ b (mod n_2)
        //
        // By Chinese Remainder Theorem:
        //     K ≡ a * m_1 * y_1 + b * m_2 * y_2 (mod n_1 * n_2)
        //       ≡ c_1 * n_1 * n_2 * y_1 + c_1 * n_1 * n_2 * y_1 (mod n_1 * n_2)
        //       ≡ n_1 * n_2 * (c_1 * y_1 + c_2 * y_2)
        // which means the value of K between 0 and n_1 * n_2 is:
        //     K ≡ (n_1 * n_2 * (c_1 * y_1 + c_2 * y_2)) % (n_1 * n_2)
        
        final int n_1 = TurretConstants.kEncoderGear1Teeth;
        final int n_2 = TurretConstants.kEncoderGear2Teeth;
        
        final int N = n_1 * n_2;

        final double c_1 = encoder1.getPosition().getValueAsDouble();
        final double c_2 = encoder2.getPosition().getValueAsDouble();
        
        final int y_1 = TurretConstants.CRTConstants.y_1;
        final int y_2 = TurretConstants.CRTConstants.y_2;

        // K; on [0, N)
        // java calculate remainder, not modulus
        // so for example -3 % 5 is not 2, but this will make it 2
        final double positionTeethRaw = (N * (c_1 * y_1 + c_2 * y_2)) % N;

        // if positive, OK, if negative, add the N
        return positionTeethRaw < 0 ? positionTeethRaw + N : positionTeethRaw;
    }

    public double setAngleFieldRelative(Rotation2d desiredRotation) {
        desiredRotation =  desiredRotation.minus(new Rotation2d(Math.toRadians(drivetrain.getHeadingBlue())));
        setAngle(desiredRotation);
        return desiredRotation.getDegrees();
    }

    public void setAngle(Rotation2d desiredRotation) {
        targetAngle = desiredRotation.getDegrees();
         

        // (-0.5, 0.5]
        final double desiredPositionRotations = desiredRotation.getDegrees() / 360;

        final double currentPositionRotations = turretMotor.getPosition();

        double optimizedDesiredPositionRotations = desiredPositionRotations;

        // the direction (+ or -) needed to find the optimized position
        double direction = Math.signum(currentPositionRotations - optimizedDesiredPositionRotations);
        double bestDiff = Math.abs(currentPositionRotations - optimizedDesiredPositionRotations);

        if (direction == 0) {
            turretMotor.setPositionDutyCycle(optimizedDesiredPositionRotations, 0);
            return;
        }
        
        for (
            double proposed = optimizedDesiredPositionRotations + direction * 1.0;

            // if direction is positive, constrained by max
            // if direction is negative, constrained by min
            (direction != 1.0 || proposed < TurretConstants.kMaxPositionRotations) &&
            (direction != -1.0 || proposed > TurretConstants.kMinPositionRotations);

            proposed += direction * TurretConstants.kTurretGearTeeth
        ) {
            double diff = Math.abs(currentPositionRotations - proposed);
            // as soon as the difference increases from going higher,
            // we have found the best one
            if (diff > bestDiff)
                break;
            optimizedDesiredPositionRotations = proposed;
            bestDiff = diff;
        }
        
        SmartDashboard.putNumber("Turret desired position", optimizedDesiredPositionRotations);

        kRv = SmartDashboard.getNumber("Turret kRv", kRv);
        kRs = SmartDashboard.getNumber("Turret kRs", kRs);
        
        double yawRate = Math.toDegrees(Drivetrain.getInstance().getYawRate());
        // NOTE: yaw rate returns radians
        // kRv is velocity feedfoward term, proportional to angular velocity
        // kRs is static feedforward term with same sign as the robot's angular velocity
        // add these two, then negate because turret needs to turn opposite direction of drivetrain
        double ff = -(kRv*yawRate + Math.signum(yawRate)*kRs);

        turretMotor.setPositionMotionMagicTorqueCurrentFOC(optimizedDesiredPositionRotations, ff);
    }

    public double getAngle() {
        return turretMotor.getPosition()*360;
    }

    public double getTargetAngle(){
        return targetAngle;
    }

    @Override
    public void periodic() {
        if (TurretConstants.ZEROING_MODE) {
            double position1 = encoder1.getAbsolutePosition().getValueAsDouble();
            double position2 = encoder2.getAbsolutePosition().getValueAsDouble();
            System.out.println("public static final double ZEROING_MODE = false;");
            System.out.println("public static final double kEncoder1MagnetOffset = " + -position1 + ";");
            System.out.println("public static final double kEncoder2MagnetOffset = " + -position2 + ";");
            System.out.println("-");
            return;
        }

        // targetAngle = SmartDashboard.getNumber("Turret target angle", 0);
        // SmartDashboard.putNumber("heading", drivetrain.getHeadingBlue());
        // SmartDashboard.putNumber("Turret current position", turretMotor.getPosition());
        // SmartDashboard.putNumber("Turret CRT position", getCurrentPositionTeethRaw() - TurretConstants.kZeroPositionTeethRaw);
        
        // double robotTarget = setAngleFieldRelative(Rotation2d.fromDegrees(targetAngle));
        // SmartDashboard.putNumber("Turret robot target", robotTarget);
        // SmartDashboard.putNumber("Turret error", Math.IEEEremainder(robotTarget - turretMotor.getPosition() * 360, 360));
    }
}
