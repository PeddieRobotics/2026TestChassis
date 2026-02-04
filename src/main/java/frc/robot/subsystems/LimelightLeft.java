package frc.robot.subsystems;

import frc.robot.utils.Constants.CameraConstants;

public class LimelightLeft extends Limelight {
    private static LimelightLeft limelightleft;
    private LimelightLeft(){
        super(CameraConstants.kLeftCamName, false);
    }
    
    public static LimelightLeft getInstance(){
        if(limelightleft==null)
            limelightleft = new LimelightLeft();
        return limelightleft;
    }

    @Override
    public void periodic(){
        super.periodic();
    }
}
