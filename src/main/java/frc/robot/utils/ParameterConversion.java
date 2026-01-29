package frc.robot.utils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.utils.Constants.FieldConstants;

// to convert from robo coords --> field coords and vice versa
// currently only suppports non-rotational shooting (in motion)
public class ParameterConversion {
    private Pose2d fieldToRobot; // (x,y) of the robot in field coords

    public ParameterConversion(Pose2d fieldToRobot) {
        this.fieldToRobot = fieldToRobot; 
        // all transformations of x,y in terms of turret here:
    }
    public Pose2d getFieldToRobot() {
        return fieldToRobot;
    }

    public static double robotDistanceToHubX(Pose2d robotPos){ //finds distance X
        double hubPositionX;
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == Alliance.Blue) {
            hubPositionX = FieldConstants.blueHubPositionX;
        }
        else {
            hubPositionX =  FieldConstants.redHubPositionX;
        }
        return robotPos.getX()-hubPositionX;
    }
    public static double robotDistanceToHubY(Pose2d robotPos){ //finds distance Y
        double hubPositionY;
        if (DriverStation.getAlliance().isEmpty() || DriverStation.getAlliance().get() == Alliance.Blue) {
            hubPositionY = FieldConstants.blueHubPositionY;
        }
        else {
            hubPositionY =  FieldConstants.redHubPositionY;
        }
        return robotPos.getY()-hubPositionY;
    }
}
