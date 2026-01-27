package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.TurretConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.OI;

public class Turret extends SubsystemBase {
    private Kraken turretMotor;
    private CANcoder encoder1, encoder2;
    private CANBus canbus;
    private PIDController PIDController;
    private static Turret turret;
    private static OI controller;

    public Turret() {
        canbus = new CANBus();
        turretMotor = new Kraken(TurretConstants.kTurretMotorDeviceId, canbus);
        encoder1 = new CANcoder(TurretConstants.kEncoderId1);
        encoder2 = new CANcoder(TurretConstants.kEncoderId2);

        // for convenient visual
        SmartDashboard.putBoolean("Open loop control", false);
        PIDController = new PIDController(TurretConstants.kTurretP, TurretConstants.kTurretI, TurretConstants.kTurretD);
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
        final double positionTeethRaw = (N * (c_1 * y_1 + c_2 * y_2)) % N;
        
        return positionTeethRaw;
    }

    // get the current angle of the turret in degrees
    public double getAngle() {
        // offset to avoid wrapping problem as discussed with Adam; on [-N/2, N/2)
        final double positionTeeth = getCurrentPositionTeethRaw() - TurretConstants.kZeroPositionTeeth;
        
        // ok, now we know how many gears left/right the turret is from the "forward" position
        // now we need to convert this to degrees
        // each tooth is (1 / kTurretGearTeeth) * 360
        // (which means rotating by the full number of gears gives 360 degrees, which is correct)
        // note that this has nothing to do with the range of the turret (-270 to 270)
        
        final double positionDegrees = TurretConstants.positionTeethToDegree(positionTeeth);
            
        return positionDegrees;
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
        final double desiredPositionTeethRaw = desiredPositionTeeth + TurretConstants.kZeroPositionTeeth;
        
        final double currentPositionTeethRaw = getCurrentPositionTeethRaw();
        
        // find the closest closest position to the current position
        // there's a easier way to do this, and someone can figure this out later
        double optimizedDesiredPositionTeethRaw = desiredPositionTeethRaw;

        // the direction (+ or -) needed to find the optimized position
        double direction = Math.signum(currentPositionTeethRaw - optimizedDesiredPositionTeethRaw);
        double bestDiff = Math.abs(currentPositionTeethRaw - optimizedDesiredPositionTeethRaw);

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
        
        // OK, now you can run a PID loop
        // the setpoint is the optimizedDesiredPositionTeethRaw
        // the input is the getCurrentPositionTeethRaw
        // this part is left as an exercise to the reader

        turretMotor.setVoltage(PIDController.calculate(getCurrentPositionTeethRaw(), optimizedDesiredPositionTeethRaw));
    }

    public void setPercentOutput(double percentAngle) {
        turretMotor.setPercentOutput(percentAngle);
    }

    @Override
    public void periodic() {
        // do someting
        if (SmartDashboard.getBoolean("Open loop control", false))
            setPercentOutput(controller.getStrafe());
        
        SmartDashboard.putNumber("turret angle", getAngle());
    }
}
