package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot26.subsystems.Hopper;

public class StartShoot extends Command {

    Shooter shooter;
    Hopper hopper;

    public StartShoot(Shooter shooter, Hopper hopper) {
        this.shooter = shooter;
        this.hopper = hopper;
    }

    @Override
    public void initialize() {
        shooter.flywheelEnabled = true;
    }

    @Override
    public void execute() {
        hopper.start();
    }

    @Override
    public boolean isFinished() {
        return shooter.flywheelEnabled == true;
    }

    @Override
    public void end(boolean inturrupted) {
    }
}
