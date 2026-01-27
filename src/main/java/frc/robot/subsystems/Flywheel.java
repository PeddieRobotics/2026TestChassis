package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;

public class Flywheel extends SubsystemBase {
    private static Flywheel flywheel;
    private Kraken motor1, motor2;
    
    private double rpm = 0;

    private Flywheel() {
        CANBus canbus = new CANBus("rio");

        motor1 = new Kraken(58, canbus);
        motor2 = new Kraken(59, canbus);

        motor1.setSupplyCurrentLimit(40);
        motor2.setSupplyCurrentLimit(40);

        motor1.setVelocityConversionFactor(1);
        motor2.setVelocityConversionFactor(1);
        
        motor2.setFollower(58, MotorAlignmentValue.Opposed);
        motor2.setInverted(true);

        motor1.setPIDValues(0.47, 0.09, 0, 0.7, 0, 0, 0);
        
        // SmartDashboard.putNumber("Flywheel P", 0);
        // SmartDashboard.putNumber("Flywheel I", 0);
        // SmartDashboard.putNumber("Flywheel D", 0);
        // SmartDashboard.putNumber("Flywheel S", 0);
        // SmartDashboard.putNumber("Flywheel V", 0);

        SmartDashboard.putNumber("Flywheel RPM", rpm = 0);
    }
    
    public static Flywheel getInstance() {
        if (flywheel == null)
            flywheel = new Flywheel();
        return flywheel;
    }
    
    @Override
    public void periodic() {
        // double P = SmartDashboard.getNumber("Flywheel P", 0);
        // double I = SmartDashboard.getNumber("Flywheel I", 0);
        // double D = SmartDashboard.getNumber("Flywheel D", 0);
        // double S = SmartDashboard.getNumber("Flywheel S", 0);
        // double V = SmartDashboard.getNumber("Flywheel V", 0);
        // motor1.setPIDValues(S, V, 0, P, I, D, 0);

        double newRPM = SmartDashboard.getNumber("Flywheel RPM", rpm) / 60;
        if (newRPM != rpm)
            motor1.setVelocityVoltage(rpm = newRPM);
        
        SmartDashboard.putNumber("Flywheel 1 RPM", motor1.getRPM());
        SmartDashboard.putNumber("Flywheel 2 RPM", motor2.getRPM());
        
        SmartDashboard.putNumber("Flywheel 1 Output", motor1.getPercentOutput());
        SmartDashboard.putNumber("Flywheel 2 Output", motor2.getPercentOutput());
    }
}

