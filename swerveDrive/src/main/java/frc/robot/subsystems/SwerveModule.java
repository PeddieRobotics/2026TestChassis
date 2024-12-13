// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utils.Kraken;

public class SwerveModule extends SubsystemBase {
  private final CANcoder turnEncoder;
  private final int drivingCANID, turningCANID, CANcoderID;
  private Kraken drivingMotor, steeringMotor;
  private SwerveModuleState desiredState;
  private double moduleOffset;
  /** Creates a new ExampleSubsystem. */
  public SwerveModule(String CANbusName, int driveCANID, int turnCANID, int CANcodeID, double moduleOffset) {
    CANcoderID = CANcodeID;
    drivingCANID = driveCANID;
    turningCANID = turnCANID;
    this.moduleOffset = moduleOffset;

    drivingMotor = new Kraken(driveCANID, CANbusName);
    steeringMotor = new Kraken(turnCANID, CANbusName);
    drivingMotor.setInverted(true);
    steeringMotor.setInverted(true);
    drivingMotor.setSupplyCurrentLimit(Constants.ModuleConstants.kDriveMotorCurrentLimit);
    steeringMotor.setSupplyCurrentLimit(Constants.ModuleConstants.kTurnMotorCurrentLimit);
    drivingMotor.setBrake();
    steeringMotor.setBrake();
    drivingMotor.setEncoder(0);
    steeringMotor.setEncoder(0);

    turnEncoder = new CANcoder(CANcodeID, CANbusName);
    configureCANcoder();
    steeringMotor.setContinuousOutput(); //keeps cancoder consisten by wrapping -5,5
    steeringMotor.setFeedbackDevice(CANcodeID, FeedbackSensorSourceValue.FusedCANcoder);//what it listens to

    drivingMotor.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderVelocityFactor);
    steeringMotor.setRotorToSensorRatio(Constants.ModuleConstants.kSteerMotorReduction);
    steeringMotor.setSensorToMechanismRatio(1.0);
  }

  public void configureCANcoder(){
    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    config.MagnetSensor.MagnetOffset = -moduleOffset/(2*Math.PI);

    turnEncoder.getConfigurator().apply(config);
  }

  public double getCANcoderReading(){
    return turnEncoder.getAbsolutePosition().getValueAsDouble()*2*Math.PI; //convert from rev to radians
  }

  public SwerveModuleState getState(){
    return new SwerveModuleState(drivingMotor.getMPS(), new Rotation2d(getCANcoderReading()));
  }

  public void setDesiredState(SwerveModuleState desiredState){
    this.desiredState = desiredState;
    SwerveModuleState optimized = SwerveModuleState.optimize(desiredState, new Rotation2d(getCANcoderReading()));
    double steerAngle = optimized.angle.getRadians()/(2*Math.PI);
    
  }
  


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
