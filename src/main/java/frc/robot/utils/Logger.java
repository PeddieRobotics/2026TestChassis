package frc.robot.utils;

import java.util.HashMap;
import java.util.Map;
import java.util.Optional;



import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleArrayLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Superstructure;
import frc.robot.subsystems.ShooterStructure;
import frc.robot.subsystems.SwerveModule;

// @SuppressWarnings("unused")
public class Logger {
    private static Logger instance;

    Limelight[] limelights;

    private Superstructure superstructure;
    private Drivetrain drivetrain;
    private DataLog log = DataLogManager.getLog();
    private StringLogEntry commandEntry;

    // private List<StructPublisher<Pose2d>> limelightMT2Entry;

    public static Logger getInstance() {
        if (instance == null) {
            instance = new Logger();
        }
        return instance;
    }

    public Logger() {
        superstructure = Superstructure.getInstance();
        drivetrain = Drivetrain.getInstance();
        

        // Commands run
        commandEntry = new StringLogEntry(log, "/Commands/Commands Run");
    }

    public void logEvent(String event, boolean isStart) {
        commandEntry.append(event + (isStart ? " Started" : " Ended"));
    }

    public void updateLogs() {
        Drivetrain.getInstance().updateDrivetrainLogs();
        Superstructure.getInstance().updateSuperstructureLogs();
        ShooterStructure.getInstance().updateShooterStructureLogs();
    }

    public void logScoreEvent(int level, double elevator, double arm) {
        logEvent("Score L" + level + " with elevator " + elevator + ", arm " + arm, false);
    }
}
