package frc.robot.utils;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.PS4Controller;

public class OI {
    private static OI oi;
    private PS4Controller controller;

    public OI(){
        controller = new PS4Controller(0);
    }

    public static OI getInstance(){
        if(oi == null){
            oi = new OI();
        }
        return oi;
    }

    public double getFoward(){
        return -controller.getRawAxis(PS4Controller.Axis.kLeftY.value);
    }

    public double getStrafe(){
        return -controller.getRawAxis(PS4Controller.Axis.kLeftX.value);
    }
    
    public Translation2d getSwerveTranslation(){
        double xSpeed = getFoward();
        double ySpeed = getStrafe();

        return new Translation2d(xSpeed, ySpeed);
    }

    public double getRotation(){
        double rightRotation = controller.getRawAxis(PS4Controller.Axis.kL2.value);
        double leftRotation = controller.getRawAxis(PS4Controller.Axis.kR2.value);

        double combinedRotation = (leftRotation - rightRotation)/2;
        return combinedRotation;
    }
}
