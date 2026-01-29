package frc.robot.utils;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.utils.Constants.FieldConstants;

public class Shifts {
    public enum TeleopShift {
        TRANSITION,
        SHIFT_1,
        SHIFT_2,
        SHIFT_3,
        SHIFT_4,
        ENDGAME
    }

    private static double autoStart, teleopStart;
    public static void startAuto() {
        autoStart = Timer.getFPGATimestamp();
    }
    public static void startTeleop() {
        teleopStart = Timer.getFPGATimestamp();
    }
    
    public static boolean isActive(){
        if (DriverStation.isAutonomous() || determineShift() == TeleopShift.ENDGAME)
            return true;

        Alliance alliance = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Blue;
        String gameData = DriverStation.getGameSpecificMessage();

        // gameData is sent 3 seconds after auto
        // at that time, the HUB is definitely active (transition shift)
        if (gameData.length() == 0)
            return true;

        Alliance firstInactiveAlliance = gameData.charAt(0) == 'B' ? Alliance.Blue : Alliance.Red;

        TeleopShift shift = determineShift();

        if (shift == TeleopShift.SHIFT_2 || shift == TeleopShift.SHIFT_4)
            return alliance == firstInactiveAlliance;

        return alliance != firstInactiveAlliance;
    }

    public static TeleopShift determineShift(){
        double time = Timer.getFPGATimestamp() - teleopStart;
        if (time <= FieldConstants.kTransitionShiftEnd)
            return TeleopShift.TRANSITION;
        if (time <= FieldConstants.kShift1End)
            return TeleopShift.SHIFT_1;
        if (time <= FieldConstants.kShift2End)
            return TeleopShift.SHIFT_2;
        if (time <= FieldConstants.kShift3End)
            return TeleopShift.SHIFT_3;
        if (time <= FieldConstants.kShift4End)
            return TeleopShift.SHIFT_4;
        return TeleopShift.ENDGAME;
    }

    public static boolean isHoldThreshold(){
        if (!isActive())
            return false;

        double time = Timer.getFPGATimestamp() - teleopStart;
        TeleopShift currentShift = determineShift();

        switch (currentShift){
            case TRANSITION:
                return FieldConstants.kTransitionShiftEnd - time < FieldConstants.kHoldThreshold;
            case SHIFT_1:
                return FieldConstants.kShift1End - time < FieldConstants.kHoldThreshold;
            case SHIFT_2:
                return FieldConstants.kShift2End - time < FieldConstants.kHoldThreshold;
            case SHIFT_3:
                return FieldConstants.kShift3End - time < FieldConstants.kHoldThreshold;
            case SHIFT_4:
                return FieldConstants.kShift4End - time < FieldConstants.kHoldThreshold;
            case ENDGAME:
                return false;
        }

        return false;
    }
}
