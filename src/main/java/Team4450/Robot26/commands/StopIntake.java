package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StopIntake extends Command {
  private Intake intake;

  public StopIntake(Intake intake) {
    this.intake = intake;
  }

  public void initialize() {
    SmartDashboard.putBoolean("Intake Stopped?", false);

  }

  public void execute() {
    intake.stopIntake();
    end();
  }

  public boolean isFinished() {
    return true;

  }

  public void end() {
    SmartDashboard.putBoolean("Intake Stopped?", true);

  }
}
