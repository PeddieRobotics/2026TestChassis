package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightRight extends Limelight {
    private static LimelightRight limelightright;
    private LimelightRight(){
        super(CameraConstants.kRightCamName, false);
    }
    
    public static LimelightRight getInstance(){
        if(limelightright==null)
            limelightright = new LimelightRight();
        return limelightright;
    }

    @Override
    public void periodic(){
        super.periodic();
    }
}
