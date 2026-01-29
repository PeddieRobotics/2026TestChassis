// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Arrays;
import java.util.List;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.Constants.ScoreConstants;
import frc.robot.utils.LiveData;
import frc.robot.utils.Logger;

import static frc.robot.subsystems.Superstructure.SuperstructureState.*;

public class Superstructure extends SubsystemBase {
    private static Superstructure superstructure;

    private Hopper hopper;
    private Climber climber;
    private Intake intake;

    private SuperstructureState systemState, requestedSystemState;

    private Timer timer;

    private boolean isManualControl;

    private StringLogEntry superstructureCurrentStateEntry, superstructureRequestedStateEntry;

    public enum SuperstructureState {
        STOW,
        CLIMB_L1,
        CLIMB_L3,
        UNCLIMB,  
        TRENCH,      
    }

    public Superstructure() {
        systemState = requestedSystemState = STOW;
        intake = Intake.getInstance();
        hopper = Hopper.getInstance();
        timer = new Timer();

        // isManualControl = false;
        DataLog log = DataLogManager.getLog();

        superstructureCurrentStateEntry = new StringLogEntry(log, "/Superstructure/Current Superstructure State");
        superstructureRequestedStateEntry = new StringLogEntry(log, "/Superstructure/Requested Superstructure State");

    }
    
    public static Superstructure getInstance() {
        if (superstructure == null)
            superstructure = new Superstructure();
        return superstructure;
    }

    public void requestState(SuperstructureState request) {
        requestedSystemState = request;
    }

    public SuperstructureState getCurrentState() {
        return systemState;
    }

    public SuperstructureState getRequestedState() {
        return requestedSystemState;
    }

    public void setManualControl(boolean isManualControl){
        this.isManualControl = isManualControl;
    }

    public void updateSuperstructureLogs(){
        superstructureCurrentStateEntry.append(getCurrentState().toString());
        superstructureRequestedStateEntry.append(getRequestedState().toString());
    }

    @Override
    public void periodic() {
        switch (systemState) {
            case STOW:
                hopper.stopHopper();
                if ((requestedSystemState == CLIMB_L1 || requestedSystemState == CLIMB_L3) && climber.climbDeployed())
                    systemState = requestedSystemState;
                break;

            case CLIMB_L1: // !TODO: Climb prep state?
                // actually climb
                if (requestedSystemState == UNCLIMB)
                    systemState = requestedSystemState;
                break;

            case CLIMB_L3:
                // actually climb
                if (requestedSystemState == UNCLIMB)
                    systemState = requestedSystemState;
                break;

            case UNCLIMB:
                // unclimb
                timer.start();
                if (requestedSystemState == STOW)
                    systemState = requestedSystemState;
                if (timer.hasElapsed(2.0)) {
                    systemState = STOW;
                    timer.reset();
                }
                break;

            case TRENCH:
                timer.start();
                // trench actions
                if (requestedSystemState == STOW)
                    systemState = requestedSystemState;
                if (timer.hasElapsed(2.0)) {
                    systemState = STOW;
                    timer.reset();
                }

                break;
        }
    }
}
