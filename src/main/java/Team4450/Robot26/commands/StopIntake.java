package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StopIntake extends Command {
    private Intake intake;

    private State state;

    private static enum State {
        STOPPING, STOPPED, END
    };

    public StopIntake(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        state = State.STOPPING;

    }

    @Override
    public void execute() {
        state = State.STOPPED;
        intake.stopIntake();
        end(false);
    }

    @Override
    public boolean isFinished() {
        return state == State.END;

    }

    @Override
    public void end(boolean inturrupted) {
        SmartDashboard.putString("Intake Stopped?", "Yup");

    }
}
