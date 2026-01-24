package frc.robot.subsystems;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.TurretConstants;
import frc.robot.utils.Kraken;
import frc.robot.utils.OI;

public class Turret extends SubsystemBase {
    private Kraken turretMotor;
    private CANcoder encoder1, encoder2;
    private CANBus canbus;
    private static Turret turret;
    private static Rotation2d angle;
    private static OI controller;

    public Turret() {
        canbus = new CANBus();
        turretMotor = new Kraken(TurretConstants.kTurretMotorDeviceId, canbus);
        encoder1 = new CANcoder(TurretConstants.kEncoderId1);
        encoder2 = new CANcoder(TurretConstants.kEncoderId2);
        // for convenient visual
        SmartDashboard.putBoolean("Open loop control", false);
    }

    public static Turret getInstance() {
        if (turret == null)
            turret = new Turret();
        return turret;
    }

    // implement +/- x degrees softlimits when given
    // in progress needs cancoder displacement
    public double getAngle() {
        // use the k is congruent to ay1n2 + by2n1 (%n1*n2), k being rotations necessary
        // m2=n1, m1=n2
        // get a,b from cancoders and work from there via equation
        int circleTeethProduct = TurretConstants.kTeethPerEncoderCircle1 * TurretConstants.kTeethPerEncoderCircle2;
        double gearsTurnedA = encoder1.getPosition().getValueAsDouble();
        double gearsTurnedB = encoder2.getPosition().getValueAsDouble();
        double rawNumGears = (gearsTurnedA * circleTeethProduct - gearsTurnedB * circleTeethProduct) % (circleTeethProduct);

        // normalize angle to the range [0,420)
        return (rawNumGears<0 ? rawNumGears+420 : rawNumGears) % TurretConstants.kTeethPerTurretCircle * 360; // in degrees
    }

    // wrong probably 90% wrong by aaryan "double A" patel
    public void setAngle(Rotation2d rotation) {
        double wantedGear = rotation.getDegrees() / 360 * TurretConstants.kTeethPerTurretCircle;
        turretMotor.setEncoder(wantedGear);
    }

    public void setPercentOutput(double percentAngle) {
        turretMotor.setPercentOutput(percentAngle);
    }

    @Override
    public void periodic() {
        // do someting
        if (SmartDashboard.getBoolean("Open loop control", false)) {
            setPercentOutput(controller.getStrafe());
        }
    }
}
