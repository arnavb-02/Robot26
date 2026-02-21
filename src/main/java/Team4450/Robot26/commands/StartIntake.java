package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StartIntake extends Command {
  private Intake intake;

  public StartIntake(Intake intake) {
    this.intake = intake;
  }

  public void initialize() {
    SmartDashboard.putBoolean("Intake Running?", false);
  }

  public void execute() {
    intake.startIntakeSlow();
    end();
  }

  public boolean isFinished() {
    return true;
  }

  public void end() {
    SmartDashboard.putBoolean("Intake Running?", true);
  }
}
