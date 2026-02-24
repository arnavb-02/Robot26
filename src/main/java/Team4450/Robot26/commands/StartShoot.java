package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Shooter;
import edu.wpi.first.wpilibj2.command.Command;

public class StartShoot extends Command {

  Shooter shooter;

  public StartShoot(Shooter shooter) {
    this.shooter = shooter;
  }

  public void initialize() {
    shooter.flywheelEnabled = true;
  }

  public void execute() {
    end();
  }

  public boolean isFinished() {
    return shooter.flywheelEnabled == true;
  }

  public void end() {
  }
}
