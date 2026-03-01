package Team4450.Robot26.commands;

import Team4450.Robot26.subsystems.Intake;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StartIntake extends Command {
    private Intake intake;

    private State state;

    private static enum State {
        MOVING, INTAKE, STOP
    };

    public StartIntake(Intake intake) {
        this.intake = intake;
    }

    @Override
    public void initialize() {
        state = State.MOVING;

    }

    @Override
    public void execute() {
        state = State.INTAKE;
        intake.startIntakeSlow();
        end(false);
    }

    @Override
    public boolean isFinished() {
        return state == State.STOP;

    }

    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putString("Done with starting intake?", "Yup");

    }
}
