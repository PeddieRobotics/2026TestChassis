package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightBack extends Limelight {
    private static LimelightBack limelightback;
    private LimelightBack(){
        super(CameraConstants.kBackCamName, false);
    }
    
    public static LimelightBack getInstance(){
        if(limelightback==null)
            limelightback = new LimelightBack();
        return limelightback;
    }

    @Override
    public void periodic(){
        super.periodic();
    }
}
