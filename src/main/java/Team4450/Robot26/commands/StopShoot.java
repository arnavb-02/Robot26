package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot26.subsystems.Hopper;

public class StopShoot extends Command {

    Shooter shooter;
    Hopper hopper;

    public StopShoot(Shooter shooter, Hopper hopper) {
        this.shooter = shooter;
        this.hopper = hopper;
    }

    @Override
    public void initialize() {
        shooter.flywheelEnabled = false;
    }

    @Override
    public void execute() {
        hopper.stop();
    }

    @Override
    public boolean isFinished() {
        return shooter.flywheelEnabled = false;
    }

    @Override
    public void end(boolean inturrupted) {
    }
}
