package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;

public class Intake extends SubsystemBase {
    private static Intake intake;
    private Kraken rightMotor;

    private Intake() {
        CANBus canbus = new CANBus("rio");

        // leftMotor = new Kraken(17, canbus);
        rightMotor = new Kraken(17, canbus);

        // leftMotor.setSupplyCurrentLimit(40);
        rightMotor.setSupplyCurrentLimit(40);

        rightMotor.setInverted(true);

        SmartDashboard.putNumber("intake left percent output", 0);
        SmartDashboard.putNumber("intake right percent output", 0);
    }
    
    public static Intake getInstance() {
        if (intake == null)
            intake = new Intake();
        return intake;
    }

    public void setIntakePercentOutput(double output) {
        // leftMotor.setPercentOutput(output);
        rightMotor.setPercentOutput(output);
    }

    public void stopIntake() {
        // leftMotor.setPercentOutput(0);
        rightMotor.setPercentOutput(0);
    }
    
    @Override
    public void periodic() {
        // leftMotor.setPercentOutput(-SmartDashboard.getNumber("intake left percent output", 0));
        rightMotor.setPercentOutput(SmartDashboard.getNumber("intake right percent output", 0));
    }
}

