package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightFront extends Limelight {
    private static LimelightFront limelightfront;
    private LimelightFront(){
        super(CameraConstants.kFrontCamName, false);
    }
    
    public static LimelightFront getInstance(){
        if(limelightfront==null)
            limelightfront = new LimelightFront();
        return limelightfront;
    }

    @Override
    public void periodic(){
        super.periodic();
    }
}
