package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Shooter;
import Team4450.Robot26.subsystems.Hopper;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot extends Command {
    private Shooter shooter;
    private Hopper hopper;

    public Shoot(Shooter shooter, Hopper hopper) {
        this.shooter = shooter;
        this.hopper = hopper;
    }

    @Override
    public void initialize() {
        shooter.enabledHood();
        shooter.startFlywheel();
    }

    @Override
    public void execute() {
        if (this.shooter.flywheelAtSpeed()) {
            shooter.startInfeed();
            hopper.start();
        } else {
            shooter.stopInfeed();
            hopper.stop();
        }
    }

    @Override
    public boolean isFinished() {
        // We will always force stop the command
        return false;
    }

    @Override
    public void end(boolean interuppted) {
        shooter.distableHood();
        shooter.stopFlywheel();
        shooter.stopInfeed();
        hopper.stop();
    }
}
