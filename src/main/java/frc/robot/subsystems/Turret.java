package frc.robot.subsystems;

import java.util.Optional;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.CameraConstants;
import frc.robot.utils.Constants.TurretConstants;
import frc.robot.utils.Kraken;
// import frc.robot.utils.LimelightHelpers;
// import frc.robot.utils.OI;

public class Turret extends SubsystemBase {
    private static Turret turret;

    // private OI oi;
    // private Limelight llTurret;

    private Kraken turretMotor;
    private CANcoder encoder1, encoder2;
    private ProfiledPIDController ProfiledPIDController;
    
    private double optimizedDesiredPositionTeethRaw;
    
    private double kPV = TurretConstants.kPV;
    private double kIV = TurretConstants.kIV;
    private double kDV = TurretConstants.kDV;
    private double kSV = TurretConstants.kSV;
    private double kFFV = TurretConstants.kFFV;
    private double kPT = TurretConstants.kPT;
    private double kIT = TurretConstants.kIT;
    private double kDT = TurretConstants.kDT;
    private double kST = TurretConstants.kST;
    private double kFFT = TurretConstants.kFFT;
    private double kEpsilon = TurretConstants.kEpsilon;
    private double kVoltageMax = TurretConstants.kVoltageMax;
    private double kTorqueMax = TurretConstants.kTorqueMax; // tune this
    private double kOmegaMax = TurretConstants.kOmegaMax;

    private Field2d fieldMT2;

    public Turret() {
        // oi = OI.getInstance();
        // llTurret = LimelightTurret.getInstance();

        CANBus canbus = new CANBus();

        turretMotor = new Kraken(TurretConstants.kTurretMotorDeviceId, canbus);
        turretMotor.setStatorCurrentLimit(40);
        turretMotor.setSupplyCurrentLimit(40);
        turretMotor.setCoast();

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

        // for convenient visual

        SmartDashboard.putBoolean("Turret open loop control", false);
        SmartDashboard.putNumber("Turret voltage output", 0);
        SmartDashboard.putNumber("Turret torque current",0);
        SmartDashboard.putBoolean("Turret angle field relative ?!",false);

        ProfiledPIDController = new ProfiledPIDController(kPT,kIT,kDT,new TrapezoidProfile.Constraints(kOmegaMax,kTorqueMax));

        SmartDashboard.putNumber("Turret P Torque", kPT);
        SmartDashboard.putNumber("Turret I Torque", kIT);
        SmartDashboard.putNumber("Turret D Torque", kDT);
        SmartDashboard.putNumber("Turret S Torque", kST);
        SmartDashboard.putNumber("Turret FF Torque", kFFT);
        SmartDashboard.putNumber("Turret Epsilon", kEpsilon);
        SmartDashboard.putNumber("Turret VoltageMax", kVoltageMax);
        SmartDashboard.putNumber("Turret TorqueMax",kTorqueMax);

        
        SmartDashboard.putNumber("Turret target angle", 0);
        optimizedDesiredPositionTeethRaw = TurretConstants.kZeroPositionTeethRaw;

        fieldMT2 = new Field2d();
        SmartDashboard.putData("Turret LL estimated pose", fieldMT2);
    }

    public static Turret getInstance() {
        if (turret == null)
            turret = new Turret();
        return turret;
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

    // get the current angle of the turret in degrees
    public double getAngle() {
        return getAngle(getCurrentPositionTeethRaw());
    }

    public double getAngle(double positionTeethRaw) {
        // offset to avoid wrapping problem as discussed with Adam; on [-N/2, N/2)
        final double positionTeeth = positionTeethRaw - TurretConstants.kZeroPositionTeethRaw;
        
        // ok, now we know how many gears left/right the turret is from the "forward" position
        // now we need to convert this to degrees
        // each tooth is (1 / kTurretGearTeeth) * 360
        // (which means rotating by the full number of gears gives 360 degrees, which is correct)
        // note that this has nothing to do with the range of the turret (-270 to 270)
        
        final double positionDegrees = TurretConstants.positionTeethToDegree(positionTeeth);
            
        return positionDegrees;
    }

    public void setAngleFieldRelative(Rotation2d desiredRotation) {
        setAngle(desiredRotation.minus(Drivetrain.getInstance().getHeadingRotation2d()));
    }

    public void setAngle(Rotation2d desiredRotation) {
        // I'm actually not sure about this part, please let me know if you have a problem

        // We will go from: desiredRotation -> ... -> desiredPositionTeethRaw
        // desiredPositionTeethRaw plus any integer multiple of the total number of teeth in the main gear
        // will result in the same position
        // we can calculate the "optimizedDesiredPositionTeethRaw" which is still "the same"
        // position as the desiredPositionTeethRaw, but which is closer to the current position
        // AND is not outside the range of the turret

        // (-180, 180]
        final double desiredPositionDegrees = desiredRotation.getDegrees();

        // (-kTurretGearTeeth / 2, kTurretGearTeeth / 2]
        final double desiredPositionTeeth = TurretConstants.positionDegreeToTeeth(desiredPositionDegrees);
        
        // this is always positive, because
        // kZeroPositionTeeth = n_1 * n_2 / 2 > kTurretGearTeeth / 2 (if the turret is well designed!)
        // (in our case: kZeroPositionTeeth = 20 * 21 / 2 = 210 > 100 = 200 / 2 = kTurretGearTeeth / 2)
        final double desiredPositionTeethRaw = desiredPositionTeeth + TurretConstants.kZeroPositionTeethRaw;
        
        final double currentPositionTeethRaw = getCurrentPositionTeethRaw();
        
        // find the closest closest position to the current position
        // there's a easier way to do this, and someone can figure this out later
        double optimizedDesiredPositionTeethRaw = desiredPositionTeethRaw;

        // the direction (+ or -) needed to find the optimized position
        double direction = Math.signum(currentPositionTeethRaw - optimizedDesiredPositionTeethRaw);
        double bestDiff = Math.abs(currentPositionTeethRaw - optimizedDesiredPositionTeethRaw);

        if (direction == 0)
            return;

        for (
            double proposed = optimizedDesiredPositionTeethRaw + direction * TurretConstants.kTurretGearTeeth;

            // if direction is positive, constrained by max
            // if direction is negative, constrained by min
            (direction != 1.0 || proposed < TurretConstants.kMaxPositionTeethRaw) &&
            (direction != -1.0 || proposed > TurretConstants.kMinPositionTeethRaw);

            proposed += direction * TurretConstants.kTurretGearTeeth
        ) {
            double diff = Math.abs(currentPositionTeethRaw - proposed);
            // as soon as the difference increases from going higher,
            // we have found the best one
            if (diff > bestDiff)
                break;
            optimizedDesiredPositionTeethRaw = proposed;
            bestDiff = diff;
        }
        
        this.optimizedDesiredPositionTeethRaw = optimizedDesiredPositionTeethRaw;
        
        // OK, now you can run a PID loop
        // the setpoint is the optimizedDesiredPositionTeethRaw
        // the input is the getCurrentPositionTeethRaw
        // this part is left as an exercise to the reader
    }

    public void setPercentOutput(double output) {
        turretMotor.setPercentOutput(output);
    }

    public void setVoltage(double currentPositionTeethRaw, double voltageCCWPlus) {
        // by default, positive voltage is clockwise, but we do not want it to be
        double voltageCWPlusForKraken = -voltageCCWPlus;
        
        // if above the "soft limit" only allow movement in direction to unwind turret
        
        // below min position = gone all the way clockwise, only counterclockwise allowed
        if (currentPositionTeethRaw <= TurretConstants.kMinPositionTeethRaw) {
            turretMotor.setVoltage(voltageCCWPlus >= 0 ? voltageCWPlusForKraken : 0);
            return;
        }

        // above max position = gone all the way counterclockwise, only clockwise allowed
        if (currentPositionTeethRaw >= TurretConstants.kMaxPositionTeethRaw) {
            turretMotor.setVoltage(voltageCCWPlus <= 0 ? voltageCWPlusForKraken : 0);
            return;
        }
        turretMotor.setVoltage(voltageCWPlusForKraken);
    }
    public void setTorque(double currentPositionTeethRaw, double torqueCCWPlus) {
        double torqueCWPlusForKraken = -torqueCCWPlus;
        
        // if above the "soft limit" only allow movement in direction to unwind turret
        
        // below min position = gone all the way clockwise, only counterclockwise allowed
        if (currentPositionTeethRaw <= TurretConstants.kMinPositionTeethRaw) {
            turretMotor.setTorqueCurrentFOC(torqueCCWPlus >= 0 ? torqueCWPlusForKraken : 0);
            return;
        }
        // above max position = gone all the way counterclockwise, only clockwise allowed
        if (currentPositionTeethRaw >= TurretConstants.kMaxPositionTeethRaw) {
            turretMotor.setTorqueCurrentFOC(torqueCCWPlus <= 0 ? torqueCWPlusForKraken : 0);
            return;
        }
        turretMotor.setTorqueCurrentFOC(torqueCWPlusForKraken);
    }
    
    // private void handleLimelight(double currentAngle) {
    //     // pretend this is zero, note this is NOT the camera pose in robot space
    //     double gyroAngle = 0;
    //     LimelightHelpers.SetRobotOrientation(
    //         LimelightConstants.kTurretLimelightName,
    //         gyroAngle, 0,
    //         0, 0,
    //         0, 0
    //     );
        
    //     // configure in meters
    //     double llRadius = 0.1;

    //     // consider if need negative signs
    //     double yawRadians = Math.toRadians(currentAngle);
    //     double llForwardOffset = llRadius * Math.cos(yawRadians);
    //     double llSideOffset = llRadius * Math.sin(yawRadians);
    //     LimelightHelpers.setCameraPose_RobotSpace(
    //         LimelightConstants.kTurretLimelightName,
    //         llForwardOffset, llSideOffset, 0,
    //         // consider if this is CW+ or CCW+, degree or radian
    //         0, 0, currentAngle
    //     );

    //     Optional<Pose2d> estimatedPoseMT2 = llTurret.getEstimatedPoseMT2();
    //     if (estimatedPoseMT2.isPresent())
    //         fieldMT2.setRobotPose(estimatedPoseMT2.get());
    // }

    @Override
    public void periodic() {
        kPT = SmartDashboard.getNumber("Turret P Torque", 0);
        kIT = SmartDashboard.getNumber("Turret I Torque", 0);
        kDT = SmartDashboard.getNumber("Turret D Torque", 0);
        kST = SmartDashboard.getNumber("Turret S Torque", 0);
        kFFT = SmartDashboard.getNumber("Turret FF Torque", 0);
        kEpsilon = SmartDashboard.getNumber("Turret Epsilon", 0);
        kVoltageMax = SmartDashboard.getNumber("Turret VoltageMax", 0);
        kTorqueMax = SmartDashboard.getNumber("Turret TorqueMax", 0);
        
        ProfiledPIDController.setPID(kPT, kIT, kDT);

        SmartDashboard.putNumber("Turret encoder 1", encoder1.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("Turret encoder 2", encoder2.getAbsolutePosition().getValueAsDouble());

        double currentPositionTeethRaw = getCurrentPositionTeethRaw();
        SmartDashboard.putNumber("Turret position teeth raw", currentPositionTeethRaw);

        double currentAngle = getAngle(currentPositionTeethRaw);
        SmartDashboard.putNumber("Turret angle", currentAngle);

        if (SmartDashboard.getBoolean("Turret open loop control", false)) {
            // setPercentOutput(0.1 * oi.getForward());
            //setVoltage(currentPositionTeethRaw, SmartDashboard.getNumber("Turret voltage output", 0));
            setTorque(currentPositionTeethRaw, SmartDashboard.getNumber("Turret torque current", 0));
            return;
        }
        
        double targetAngle = SmartDashboard.getNumber("Turret target angle", 0);
        
        if(SmartDashboard.getBoolean("Turret angle field relative ?!",false))
            setAngleFieldRelative(Rotation2d.fromDegrees(targetAngle)); 
        else
            setAngle(Rotation2d.fromDegrees(targetAngle));

        SmartDashboard.putNumber("Turret optimized target position", optimizedDesiredPositionTeethRaw);

        double error = optimizedDesiredPositionTeethRaw - currentPositionTeethRaw;
        
        double voltageOut = 0;
        double torqueOut = 0;
        if (Math.abs(error) > kEpsilon) {
            // voltageOut = PIDController.calculate(currentPositionTeethRaw, optimizedDesiredPositionTeethRaw);
            // voltageOut += Math.signum(error) * (kFF + kS);
            // voltageOut = Math.min(Math.abs(voltageOut), kVoltageMax) * Math.signum(voltageOut);
            torqueOut = ProfiledPIDController.calculate(currentPositionTeethRaw, optimizedDesiredPositionTeethRaw);
            torqueOut += Math.signum(error)*(kFFT+kST);
            torqueOut = Math.min(Math.abs(torqueOut),kTorqueMax*Math.signum(torqueOut));
        }

        setTorque(currentPositionTeethRaw, torqueOut);

        SmartDashboard.putNumber("Turret error", error);
        SmartDashboard.putNumber("Turret voltage out", voltageOut); 
        SmartDashboard.putNumber("Turret torque current",torqueOut);       
        // handleLimelight(currentAngle);
    }
}
