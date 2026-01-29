// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.HopperConstants;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;


public class Hopper extends SubsystemBase {
    private static Hopper hopper;

    private Kraken hopperMotor;

    public Hopper(){
    hopperMotor = new Kraken(RobotMap.HOPPER_MOTOR_CAN_ID, new CANBus(RobotMap.CANIVORE_NAME));

    hopperMotor.setSupplyCurrentLimit(IntakeConstants.kHopperCurrentLimit);
    hopperMotor.setBrake();

    SmartDashboard.putBoolean("Hopper Percent Output", false);
    SmartDashboard.putNumber("Hopper Motor Percent Output", 0);
    }

    public static Hopper getInstance(){
        if (hopper == null){
            hopper = new Hopper();
        }
        return hopper;
    }

  public void runHopperGroundIntake(){
    //indexing (but not shooting logic) here
    setHopper(HopperConstants.kGroundIntakeHopperSpeed);
  }

  public void runHopperOuttake(){
    //indexing (but not shooting logic) here
    setHopper(HopperConstants.kOuttakeHopperSpeed);
  }

  public void runHopperShoot(){
    //indexing (but not shooting logic) here
    setHopper(HopperConstants.kShootHopperSpeed);
  }


  public void feedFlywheel() {
    setHopper(HopperConstants.kFeedFlywheel);
  }

  public void setHopper(double speed) {
    hopperMotor.setPercentOutput(speed);
  }

  public void stopHopper() {
    hopperMotor.setPercentOutput(0);
  }

  public boolean hasGamePiece(){
    return false;
  }

  public boolean isGamePieceIndexed(){
    return false;
  }

  public double getMotorSupplyCurrent(){
    return hopperMotor.getSupplyCurrent();
  }
  
    
    @Override
    public void periodic() {
    }
}
