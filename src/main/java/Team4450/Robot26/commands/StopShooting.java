package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot26.subsystems.Hopper;

public class StopShooting extends Command {

    Shooter shooter;
    Hopper hopper;

    public StopShooting(Shooter shooter, Hopper hopper) {
        this.shooter = shooter;
        this.hopper = hopper;
    }

    @Override
    public void end(boolean interuppted) {
        shooter.distableHood();
        shooter.stopFlywheel();
        shooter.stopInfeed();
        hopper.stop();
    }
}
