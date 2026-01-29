// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import com.ctre.phoenix6.CANBus;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.IntakeConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.RobotMap;


public class Intake extends SubsystemBase {
  private static Intake intake;

  private Kraken intakeMotor, retractMotor;

  public Intake(){
      intakeMotor = new Kraken(RobotMap.INTAKE_MOTOR_ID, new CANBus(RobotMap.CANIVORE_NAME));
      retractMotor = new Kraken(RobotMap.RETRACT_MOTOR_ID, new CANBus(RobotMap.CANIVORE_NAME));

  }

  public static Intake getInstance(){
      if (intake == null){
          intake = new Intake();
      }
      return intake;
  }

  public void setIntake(double speed) {
    intakeMotor.setPercentOutput(speed);  
  }
  
  public void stopIntake() {
    intakeMotor.setPercentOutput(0);
  }

  public void runIntake(){
    setIntake(IntakeConstants.kIntakeSpeed);
  }

  public void runIntakeFeed(){
    setIntake(IntakeConstants.kIntakeFeedSpeed);
  }

  public void reverseIntake(){
    setIntake(-IntakeConstants.kIntakeSpeed);
  }

  //@param position - cancoder units
  public void setIntakePositionVoltage(double position){
    retractMotor.setPositionVoltage(position);
  }

  public double getIntakeSpeed(){
    return intakeMotor.getPercentOutput();
  }

  public boolean hasGamepiece(){
    return false;
  }

  public double getMotorSupplyCurrent() {
    return 0;
  }



    @Override
    public void periodic() {
    }
}
