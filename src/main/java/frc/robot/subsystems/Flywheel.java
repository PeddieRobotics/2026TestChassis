package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;

public class Flywheel extends SubsystemBase {
    private static Flywheel flywheel;
    private Kraken motor1, motor2;

    private Flywheel() {
        CANBus canbus = new CANBus("rio");

        motor1 = new Kraken(58, canbus);
        motor2 = new Kraken(59, canbus);

        motor1.setSupplyCurrentLimit(40);
        motor2.setSupplyCurrentLimit(40);

        SmartDashboard.putNumber("Flywheel speed", 0);
    }
    
    public static Flywheel getInstance() {
        if (flywheel == null)
            flywheel = new Flywheel();
        return flywheel;
    }
    
    @Override
    public void periodic() {
        double percent = SmartDashboard.getNumber("Flywheel speed", 0);
        
        motor1.setPercentOutput(percent);
        motor2.setPercentOutput(-percent);
    }
}

