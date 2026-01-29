// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ClimberConstants;
import frc.robot.utils.OperatorOI;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;


public class Climber extends SubsystemBase {
    private static Climber climber;

    private Kraken climberMotor;

    public Climber(){
        climberMotor = new Kraken(RobotMap.CLIMBER_MAIN_ID, new CANBus(RobotMap.CANIVORE_NAME));

        climberMotor.setEncoder(0.0);

        climberMotor.setSupplyCurrentLimit(40.0);
        climberMotor.setStatorCurrentLimit(40.0);

        climberMotor.setSoftLimits(true, 300, 0);
    }

    public static Climber getInstance(){
        if (climber == null){
            climber = new Climber();
        }
        return climber;
    }

    public void retractClimber(){
        double speed = -0.75;
        if (climbRetracted()){
           setSpeed(0.0); 
        } else {
            setSpeed(speed);
        }
    }

    public void deployClimber(){
        if(climbDeployed()){
            setSpeed(0.0);
        } else {
            setSpeed(ClimberConstants.kClimberDeployedPercentOutput);
        }
    }

    public boolean climbRetracted(){
        return getClimberPosition() < ClimberConstants.kClimberRetractedPosition;

    }
    
    public boolean climbDeployed(){
        return getClimberPosition() > ClimberConstants.kClimberDeployedPosition;
    }

    /**
     * Sets leftClimberMotor spe
     * ed to a designated percent output (open loop control)
     * ex: input of 0.5 will run 50% of its max speed forward
     * 
     * @param speed - Percent of leftClimberMotor's speed [-1.0, 1.0]
     */
    public void setSpeed(double speed) {
        climberMotor.setPercentOutput(speed);
    }

    /**
     * @return returns climber motor supply current draw (amps)
     */
    public double getClimberSupplyCurrent() {
        return climberMotor.getSupplyCurrent();
    }
    public double getClimberStatorCurrent() {
        return climberMotor.getStatorCurrent();
    }

    /**
     * @return returns temperature of climber motor in celsius
     */
    public double getClimberTemperature() {
        return climberMotor.getMotorTemperature();
    }

    /**
     * @return returns position reading of leftClimberMotor encoder (mechanism rotations)
     */
    public double getClimberPosition() {
        return climberMotor.getPosition();
    }

    @Override
    public void periodic() {
        if(SmartDashboard.getBoolean("Climber: Open Loop Control", false)){
            climberMotor.setPercentOutput(OperatorOI.getInstance().getRightForward());
        }
        SmartDashboard.putBoolean("Climber: deployed", climbDeployed());
        SmartDashboard.putBoolean("Climber: retracted", climbRetracted());
        SmartDashboard.putNumber("Climber: position", getClimberPosition());

    }
}
