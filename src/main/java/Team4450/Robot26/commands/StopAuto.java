package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot26.subsystems.Drivebase;

public class StopAuto extends Command {
    Drivebase drivebase;

    public StopAuto(Drivebase drivebase) {
        this.drivebase = drivebase;
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        drivebase.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean inturrupted) {
    }
}
