// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.utils.Constants.HoodConstants;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Constants.ShooterConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;


public class Hood extends SubsystemBase {
  private static Hood hood;
  private Boolean homing = false;

  private Kraken hoodMotor;

  public Hood(){
        hoodMotor = new Kraken(RobotMap.HOOD_MOTOR_ID, new CANBus(RobotMap.CANIVORE_BUS));
        
    }

  public static Hood getInstance(){
      if (hood == null){
          hood = new Hood();
      }
      return hood;
  }

  public void setHoodAngle(double angle) {
      if (angle<HoodConstants.kHoodMaxAngle && angle>HoodConstants.kHoodMinAngle) {
        hoodMotor.setEncoder(angleToEncoder(angle));
      }
  }

  public double getHoodAngle(){
    return encoderToAngle(hoodMotor.getPosition());
    
  }

  public void setHoming() {
    homing = true;

  }

  public double encoderToAngle(double encoderTicks) {
    return encoderTicks/HoodConstants.kHoodCPR % 360;
  }

  public double angleToEncoder(double angle) {
    return angle/360*HoodConstants.kHoodCPR;
  }

  public double getMotorSupplyCurrent() {
    return 0;
  }
    
    @Override
    public void periodic() {
    }
}
