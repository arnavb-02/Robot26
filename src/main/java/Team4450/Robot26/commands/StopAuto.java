package Team4450.Robot26.commands;

import edu.wpi.first.wpilibj2.command.Command;
import Team4450.Robot26.subsystems.Drivebase;

public class StopAuto extends Command {
  Drivebase drivebase;

  public StopAuto(Drivebase drivebase) {
    this.drivebase = drivebase;
  }

  public void initialize() {
  }

  public void execute() {
    drivebase.stop();
  }

  public boolean isFinished() {
    return true;
  }

  public void end() {
  }
}
