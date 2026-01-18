package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TestSubsystem extends SubsystemBase {
    private TalonFX testMotor = new TalonFX(Constants.FLYWHEEL_MOTOR_CAN_ID);
    // Flywheel state (RPM units)
    private double targetRpm = 0.0;
    // most recent measured RPM (for accessor/telemetry)
    private double currentRpm = 0.0;
    // keep last measured velocity (rotations per second) to estimate accel for feedforward
    private double lastRps = 0.0;
    private double maxRpm = Constants.FLYWHEEL_MAX_THEORETICAL_RPM;

    public TestSubsystem() {
        // Configure motor neutral mode and closed-loop gains for onboard control.
        testMotor.setNeutralMode(NeutralModeValue.Brake);

        // Configure TalonFX slot gains & motion settings
        TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
        var slot0 = talonFXConfigs.Slot0;
        slot0.kS = Constants.FLYWHEEL_kS;
        slot0.kV = Constants.FLYWHEEL_kV;
        slot0.kA = Constants.FLYWHEEL_kA;
        slot0.kP = Constants.FLYWHEEL_kP;
        slot0.kI = Constants.FLYWHEEL_kI;
        slot0.kD = Constants.FLYWHEEL_kD;

        // Motion Magic velocity configs (only used if you enable Motion Magic Velocity)
        talonFXConfigs.MotionMagic.MotionMagicAcceleration = Constants.FLYWHEEL_MOTION_ACCEL_RPMS / 60.0; // convert RPM/s to RPS/s
        talonFXConfigs.MotionMagic.MotionMagicJerk = Constants.FLYWHEEL_MOTION_JERK;

        // Apply configuration to the motor (blocking call)
        testMotor.getConfigurator().apply(talonFXConfigs);
    }

    @Override
    public void periodic() {
        double dt = Constants.ROBOT_PERIOD_SEC;

    // Read actual rotor velocity from the Talon (rotations per second)
    // getRotorVelocity() returns a StatusSignal<AngularVelocity> so extract a primitive
    var rotorVelSignal = testMotor.getRotorVelocity();
    // Refresh the signal value (non-blocking request for most recent value)
    rotorVelSignal.refresh();
    // Use StatusSignal's primitive extractor. getValueAsDouble() returns the value
    // in the sensor's base units (rotations per second for rotor velocity).
    double measuredRps = rotorVelSignal.getValueAsDouble();
    double measuredRpm = measuredRps * 60.0;

    // store for accessor/telemetry
    this.currentRpm = measuredRpm;

        // Convert target RPM to rotations per second for control request
        double targetRps = targetRpm / 60.0;

        // Estimate acceleration (rps/s) using last measured value for feedforward
        double accelRpsPerSec = 0.0;
        if (dt > 0) accelRpsPerSec = (targetRps - lastRps) / dt;

        // Build a VelocityVoltage control request (uses slot 0 gains configured above)
        VelocityVoltage vRequest = new VelocityVoltage(0).withSlot(Constants.FLYWHEEL_PID_SLOT)
                .withVelocity(targetRps);

        // Compute feedforward (Volts)
        double ffVolts = Constants.FLYWHEEL_kS * Math.signum(targetRps)
                + Constants.FLYWHEEL_kV * targetRps
                + Constants.FLYWHEEL_kA * accelRpsPerSec;
        vRequest = vRequest.withFeedForward(ffVolts);

        // Send the closed-loop request to the Talon (the device will maintain velocity onboard)
        testMotor.setControl(vRequest);

        // Telemetry
        double percent = 0.0;
        if (maxRpm > 0.0) percent = measuredRpm / maxRpm;
        if (percent > 1.0) percent = 1.0;
        if (percent < -1.0) percent = -1.0;

        SmartDashboard.putNumber("Flywheel/TargetRPM", targetRpm);
        SmartDashboard.putNumber("Flywheel/MeasuredRPM", measuredRpm);
        SmartDashboard.putNumber("Flywheel/FeedForwardVolts", ffVolts);
        SmartDashboard.putNumber("Flywheel/PercentOutApprox", percent);

        // remember last measured rps for next tick
        lastRps = measuredRps;
    }

    public void start() {
        // Start by setting the target RPM from constants. periodic() will ramp the motor to it.
        this.targetRpm = Constants.FLYWHEEL_DEFAULT_TARGET_RPM;
    }

    public void stop() {
        // Ramp down to zero
        this.targetRpm = 0.0;
    }

    // Optional programmatic control
    public void setTargetRpm(double rpm) { this.targetRpm = rpm; }
    public double getTargetRpm() { return this.targetRpm; }
    public double getCurrentRpm() { return this.currentRpm; }
}
