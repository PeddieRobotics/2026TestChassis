package frc.robot.commands;

import frc.robot.subsystems.Hopper;
import frc.robot.subsystems.Flywheel;
import frc.robot.subsystems.ShooterStructure;
import frc.robot.subsystems.ShooterStructure.ShooterStructureState;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class OverrideScore extends Command {

    private Flywheel shooter;
    private Hopper hopper;
    private ShooterStructure shooterStructure;

    public OverrideScore() {
        hopper = Hopper.getInstance();
        shooter = Flywheel.getInstance();
        shooterStructure = ShooterStructure.getInstance();
        addRequirements(hopper, shooter);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // shooter.setScoreSpeed();
        hopper.runHopperShoot();
        shooterStructure.setOperatorOverride(true);
        shooterStructure.overrideState(ShooterStructureState.SCORE);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        hopper.stopHopper();
        shooterStructure.setOperatorOverride(false);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}