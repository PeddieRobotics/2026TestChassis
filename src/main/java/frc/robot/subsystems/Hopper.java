package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Kraken;

public class Hopper extends SubsystemBase {
    private static Hopper hopper;
    private Kraken spindexerMotor, greenWheelMotor;

    private Hopper() {
        CANBus canbus = new CANBus("rio");

        spindexerMotor = new Kraken(48, canbus);
        greenWheelMotor = new Kraken(49, canbus);

        spindexerMotor.setSupplyCurrentLimit(40);
        greenWheelMotor.setSupplyCurrentLimit(40);

        SmartDashboard.putNumber("spindexer Percent Output",0);
        SmartDashboard.putNumber("green wheel Percent Output", 0);
    }
    
    public static Hopper getInstance() {
        if (hopper == null)
            hopper = new Hopper();
        return hopper;
    }
    
    @Override
    public void periodic() {
        spindexerMotor.setPercentOutput(-SmartDashboard.getNumber("spindexer Percent Output", 0));
        greenWheelMotor.setPercentOutput(SmartDashboard.getNumber("green wheel Percent Output", 0));
    }
}

