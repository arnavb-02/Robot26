package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class StopShoot extends Command {

  Shooter shooter;

  public StopShoot(Shooter shooter) {
    this.shooter = shooter;
  }

  public void initialize() {
    shooter.flywheelEnabled = false;
  }

  public void execute() {
    end();
  }

  public boolean isFinished() {
    return shooter.flywheelEnabled = false;
  }

  public void end() {
  }
}
