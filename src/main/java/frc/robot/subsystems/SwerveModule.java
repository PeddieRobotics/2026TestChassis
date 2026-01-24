// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.Constants;
import frc.robot.Constants.ModuleConstants;
import frc.robot.utils.Kraken;

public class SwerveModule extends SubsystemBase {
    private final CANcoder turnEncoder;
    private final int drivingCANID, turningCANID, CANcoderID;
    private Kraken drivingMotor, steeringMotor;
    private SwerveModuleState optimizedState;
    private double moduleOffset;

    /** Creates a new ExampleSubsystem. */
    public SwerveModule(CANBus canbus, int driveID, int turnID, int CANCoderID, double moduleOffset) {
        CANcoderID = CANCoderID;
        drivingCANID = driveID;
        turningCANID = turnID;
        this.moduleOffset = moduleOffset;

        drivingMotor = new Kraken(driveID, canbus);
        steeringMotor = new Kraken(turnID, canbus);
        drivingMotor.setInverted(true);
        steeringMotor.setInverted(true);
        drivingMotor.setSupplyCurrentLimit(Constants.ModuleConstants.kDriveMotorCurrentLimit);
        steeringMotor.setSupplyCurrentLimit(Constants.ModuleConstants.kTurnMotorCurrentLimit);
        drivingMotor.setBrake();
        steeringMotor.setBrake();
        drivingMotor.setEncoder(0);
        steeringMotor.setEncoder(0);
        drivingMotor.setClosedLoopRampRate(0.1);
        steeringMotor.setClosedLoopRampRate(0.1);

        turnEncoder = new CANcoder(CANCoderID, canbus);
        configureCANcoder();
        steeringMotor.setContinuousOutput(); // keeps cancoder consisten by wrapping -5,5
        steeringMotor.setFeedbackDevice(CANCoderID, FeedbackSensorSourceValue.FusedCANcoder);// what it listens to

        drivingMotor.setVelocityConversionFactor(Constants.ModuleConstants.kDriveEncoderVelocityFactor);
        steeringMotor.setRotorToSensorRatio(Constants.ModuleConstants.kSteerMotorReduction);
        steeringMotor.setSensorToMechanismRatio(1.0);

        drivingMotor.setPIDValues(
            ModuleConstants.kDrivingS, ModuleConstants.kDrivingV, ModuleConstants.kDrivingA,
            ModuleConstants.kDrivingP, ModuleConstants.kDrivingI, ModuleConstants.kDrivingD,
            ModuleConstants.kDrivingFF
        );
        steeringMotor.setPIDValues(
            ModuleConstants.kSteerS, ModuleConstants.kSteerV, ModuleConstants.kSteerA,
            ModuleConstants.kSteerP, ModuleConstants.kSteerI, ModuleConstants.kSteerD,
            ModuleConstants.kSteerFF, StaticFeedforwardSignValue.UseClosedLoopSign
        );
    }

    public void configureCANcoder() {
        CANcoderConfiguration config = new CANcoderConfiguration();
        config.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
        config.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        config.MagnetSensor.MagnetOffset = -moduleOffset;

        turnEncoder.getConfigurator().apply(config);
    }

    public double getCANcoderReading() {
        return turnEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI; // convert from rev to radians
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(drivingMotor.getMPS(), new Rotation2d(getCANcoderReading()));
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            drivingMotor.getPosition() * Constants.ModuleConstants.kDriveEncoderVelocityFactor,
            new Rotation2d(getCANcoderReading())
        );
    }

    public void setOptimizedState(SwerveModuleState desiredState) {
        this.optimizedState = desiredState;
        
        optimizedState.optimize(new Rotation2d(getCANcoderReading()));

        double desiredVelocity = optimizedState.speedMetersPerSecond;
        double desiredAngle = optimizedState.angle.getRadians() / (2 * Math.PI);
        SmartDashboard.putNumber(drivingCANID + " desired velocity", desiredVelocity);
        SmartDashboard.putNumber(turningCANID + " desired angle", desiredAngle * 2 * Math.PI);
        SmartDashboard.putNumber(CANcoderID + "current rotation", getCANcoderReading());

        drivingMotor.setVelocityVoltageWithFeedForward(desiredVelocity);
        steeringMotor.setPositionVoltageWithFeedForward(desiredAngle);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber(CANcoderID + " Cancoder name", getCANcoderReading());
        // This method will be called once per scheduler run
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }
}