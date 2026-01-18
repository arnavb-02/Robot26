package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestSubsystem extends SubsystemBase {
    // private TalonFX testMotor = new TalonFX(10);
    // // Flywheel state (RPM units)
    // private double currentRpm = 0.0;
    // private double targetRpm = 0.0;
    // private double accelRpms = Constants.FLYWHEEL_DEFAULT_ACCEL_RPMS;
    // private double maxRpm = Constants.FLYWHEEL_MAX_THEORETICAL_RPM;

    // public TestSubsystem() {
    //     testMotor.setNeutralMode(NeutralModeValue.Brake);
    // }

    // @Override
    // public void periodic() {
    //     // Ramp current RPM toward targetRpm using accelRpms (RPM per second)
    //     double dt = Constants.ROBOT_PERIOD_SEC;
    //     double maxDelta = accelRpms * dt;
    //     double delta = targetRpm - currentRpm;
    //     if (delta > maxDelta) delta = maxDelta;
    //     if (delta < -maxDelta) delta = -maxDelta;
    //     currentRpm += delta;

    //     // Convert to percent of max RPM and command motor (negative to match existing sign)
    //     double percent = 0.0;
    //     if (maxRpm > 0.0) percent = currentRpm / maxRpm;
    //     if (percent > 1.0) percent = 1.0;
    //     if (percent < -1.0) percent = -1.0;
    //     testMotor.set(-percent);

    //     // Diagnostic telemetry to verify ramp behavior and commanded outputs.
    //     SmartDashboard.putNumber("Flywheel/TargetRPM", targetRpm);
    //     SmartDashboard.putNumber("Flywheel/CurrentRPM", currentRpm);
    //     SmartDashboard.putNumber("Flywheel/AccelRPMperSec", accelRpms);
    //     SmartDashboard.putNumber("Flywheel/PercentOut", percent);
    //     SmartDashboard.putNumber("Flywheel/MaxDeltaPerTick", maxDelta);
    // }

    // public void start() {
    //     // Start by setting the target RPM from constants. periodic() will ramp the motor to it.
    //     this.targetRpm = Constants.FLYWHEEL_DEFAULT_TARGET_RPM;
    // }

    // public void stop() {
    //     // Ramp down to zero
    //     this.targetRpm = 0.0;
    // }

    // // Optional programmatic control
    // public void setTargetRpm(double rpm) { this.targetRpm = rpm; }
    // public double getTargetRpm() { return this.targetRpm; }
    // public double getCurrentRpm() { return this.currentRpm; }
}
