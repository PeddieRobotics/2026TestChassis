// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.net.InetSocketAddress;
import java.util.logging.LogManager;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;

public class Intake extends SubsystemBase {
    private static Intake intake;

    private Kraken motor1, motor2;
    private CANBus canivore_bus = new CANBus("rio");

    public Intake() {
        motor1 = new Kraken(48, canivore_bus);
        motor2 = new Kraken(17, canivore_bus);

        motor1.setStatorCurrentLimit(60);
        motor1.setSupplyCurrentLimit(60);

        motor2.setStatorCurrentLimit(60);
        motor2.setSupplyCurrentLimit(60);

        motor1.setInverted(false);
        motor2.setInverted(true);
        
        SmartDashboard.putNumber("Intake output", 0);
    }

    public static Intake getInstance() {
        if (intake == null)
            intake = new Intake();
        return intake;
    }

    public void runIntake(double output) {
        motor1.setPercentOutput(output);
        motor2.setPercentOutput(output);
    }
    public void runIntake() {
        runIntake(SmartDashboard.getNumber("Intake output", 0));
    }
    public void stopIntake() {
        motor1.setPercentOutput(0);
        motor2.setPercentOutput(0);
    }

    @Override
    public void periodic() { 
        runIntake();
    }
}
