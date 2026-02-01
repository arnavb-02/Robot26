package Team4450.Robot26.subsystems.utility;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import Team4450.Lib.Util;

import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import static edu.wpi.first.units.Units.*;

/**
 * Provides PID based velocity control for a CTRE motor using TalonFX controller.
 * Configures controller with PID parameters and runs the control loop on the
 * controller. Provides voltage, torque and Motion Magic (voltage) control methods.
 * Torque control requires Phoenix Pro license.
 * Set any parameters before calling setDesiredRPS or setDesiredRPM. Automatically
 * stops motor when robot disabled. Default parameters work with Falcon 500 and
 * should be close for Kraken. See CTRE and WPILib doc for information about the
 * parameters and how to tune this controller.
 */
public class TalonFXVelocityController extends SubsystemBase {

    private TalonFX         talon;
    private String          name;

    /* Be able to switch which control request to use based on a button press */
    /* Start at velocity 0, use slot 0 */
    private final VelocityVoltage velocityVoltage = new VelocityVoltage(0).withSlot(0);
  
    /* Start at velocity 0, use slot 1 */
    private final VelocityTorqueCurrentFOC velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);
    
    // create a Motion Magic Velocity request, voltage output
    private MotionMagicVelocityVoltage velocityMotionMagic = new MotionMagicVelocityVoltage(0);  
    
    /* Set to configured neutral mode for when we disable the motor */
    private final NeutralOut brake = new NeutralOut();
    private NeutralModeValue neutralModeValue = NeutralModeValue.Brake;
    private InvertedValue    invertedValue = InvertedValue.Clockwise_Positive;
    
    private double              desiredRotationsPerSecond = 0, driveRatio = 1;
    private MotorControlType    controlType = MotorControlType.voltage;

    /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
    private double  slot0_kS = 0.1;   // To account for friction, add 0.1 V of static feedforward
    private double  slot0_kV = 0.11;  // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
    private double  slot0_kA = 0.01;  // An acceleration of 1 rps/s requires 0.01 V output    
    private double  slot0_kP = 0.30;  // An error of 1 rotation per second results in 0.30 V output
    private double  slot0_kI = 0;     // No output for integrated error
    private double  slot0_kD = 0;     // No output for error derivative

    // Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
    private double  slot1_kS = 1.3;   // To account for friction, add 2.5 A of static feedforward
    private double  slot1_kP = 5;     // An error of 1 rotation per second results in 5 A output
    private double  slot1_kI = 0;     // No output for integrated error
    private double  slot1_kD = 0;     // No output for error derivative
        
    //DoubleConsumer setkP = kP -> setSlot0_kP(kP);
    
    public TalonFXVelocityController(TalonFX talon, String name) {
        this.talon = talon;
        this.name = name + "(" + talon.getDeviceID() + ")";

        Util.consoleLog("%s", this.name);

        SendableRegistry.addLW(this, this.name);

        RobotModeTriggers.disabled().onChange(new InstantCommand(this::stop));

        if (RobotBase.isSimulation()) PhoenixPhysicsSim.getInstance().addTalonFX(talon, 0.001);
    }

    private void configureTalon() {
        Util.consoleLog();

        TalonFXConfiguration configs = new TalonFXConfiguration();

        configs.Slot0.kS = slot0_kS;
        configs.Slot0.kV = slot0_kV;
        configs.Slot0.kA = slot0_kA;
        configs.Slot0.kI = slot0_kI; 
        configs.Slot0.kD = slot0_kD; 
        
        // Peak output of 12 volts
        configs.Voltage.withPeakForwardVoltage(Volts.of(12))
            .withPeakReverseVoltage(Volts.of(-12));

        configs.Slot1.kS = slot1_kS;
        configs.Slot1.kP = slot1_kP;
        configs.Slot1.kI = slot1_kI;
        configs.Slot1.kD = slot1_kD;

        // Peak output of 40 A
        configs.TorqueCurrent.withPeakForwardTorqueCurrent(Amps.of(40))
            .withPeakReverseTorqueCurrent(Amps.of(-40));

        /* Retry config apply up to 5 times, report if failure */
        StatusCode status = StatusCode.StatusCodeNotInitialized;

        // set Motion Magic Velocity settings
        var motionMagicConfigs = configs.MotionMagic;
        motionMagicConfigs.MotionMagicAcceleration = 400; // Target acceleration of 400 rps/s (0.25 seconds to max)
        motionMagicConfigs.MotionMagicJerk = 4000; // Target jerk of 4000 rps/s/s (0.1 seconds)
        
        configs.MotorOutput.withInverted(invertedValue).withNeutralMode(neutralModeValue);

        configs.Feedback.withSensorToMechanismRatio(1 / driveRatio);
        
        for (int i = 0; i < 5; ++i) {
            status = talon.getConfigurator().apply(configs);
            if (status.isOK()) break;
        }

        if (!status.isOK()) {
            System.out.println("Could not apply configs, error code: " + status.toString());
        }
    }
        
    /**
     * Called on every scheduler loop.
     */
    @Override
    public void periodic() {
    }

    /**
     * Called on every scheduler loop when in simulation.
     */
    @Override
    public void simulationPeriodic() {
        PhoenixPhysicsSim.getInstance().run();
    }

    /**
     * Set invert of motor direction. Default is clockwise looking at output shaft.
     * @param inverted True inverts from default. 
     * @return Itself
    */
    public TalonFXVelocityController withInverted(boolean inverted) {
        if (inverted)
            invertedValue = InvertedValue.CounterClockwise_Positive;
        else
            invertedValue = InvertedValue.Clockwise_Positive;

        return this;
    }

    /**
     * Sets neutral mode (brake/coast) of the motor.
     * @param mode Neutral mode. Defaults to brake.
     * @return Itself
     */
    public TalonFXVelocityController withNeutralMode(NeutralModeValue mode) {
        neutralModeValue = mode;

        return this;
    }

    /**
     * Set the control type to be used to control the motor. Defaults to voltage.
     * call desired*() to restart the PID control with the new value.
     * @param controlType Sets motor control type to use. 
     * @return Itself
     */
    public TalonFXVelocityController WithControlType(MotorControlType controlType) {
        this.controlType = controlType;

        return this;
    }

    /**
     * Set desired motor velocity and start motor.
     * @param rps Velocity in rotations per second.
     */
    public void desiredRPS(double rps)
    {
        desiredRotationsPerSecond = rps;

        start();
    }

    /**
     * Set desired motor velocity and start motor.
     * @param rpm Velocity in rotations per minute.
     */
    public void desiredRPM(double rpm) {
        desiredRotationsPerSecond = rpm / 60;

        start();
    }

    /**
     * Starts the motor controller with the selected control type
     * to reach and hold the desired rps.
     */
    private void start() {
        Util.consoleLog("%.2f", desiredRotationsPerSecond);

        configureTalon();
        
        if (controlType == MotorControlType.motionMagic) {
            /* Use Motion Magic control */
            talon.setControl(velocityMotionMagic.withVelocity(desiredRotationsPerSecond));
        } else if (controlType == MotorControlType.voltage) { 
            /* Use velocity voltage control */
                talon.setControl(velocityVoltage.withVelocity(desiredRotationsPerSecond));
            } else {
            /* Use velocity torque control */
                talon.setControl(velocityTorque.withVelocity(desiredRotationsPerSecond));
            }
    }

    /**
     * Stops the motor with the TalonFX controller's configured neutral mode.
     */
    public void stop() {
        Util.consoleLog();

        desiredRotationsPerSecond = 0;

        talon.setControl(brake);
    }

    /**
     * Sets the drive ratio between motor shaft and driven mechanisim. This is
     * the number of motor shaft rotations to 1 rotation of the driven mechanisim.
     * when ratio greater than one, the desired* functions set the velocity of the 
     * driven mechanisim.
     * @param ratio number of motor shaft rotations to one mecanisim rotation.
     * @return Itself
     */
    public TalonFXVelocityController withDriveRatio(double ratio) {
        driveRatio = ratio;

        return this;
    }

    /**
     * Set the kP value for voltage PID control. After changing this value
     * call desired*() to restart the PID control with the new value.
     * @param kP The proportional value. Voltage applied for error of 1 rps.
     * @return Itself
     */
    public TalonFXVelocityController withVoltagekP(double kP) {
        slot0_kP = kP;

        return this;
    }

    /**
     * Set the kI value for voltage PID control. After changing this value
     * call desired*() to restart the PID control with the new value.
     * @param kI The integral value.
     * @return Itself
     */
    public TalonFXVelocityController withVoltagekI(double kI) {
        slot0_kI = kI;

        return this;
    }

    /**
     * Set the kD value for voltage PID control. After changing this value
     * call desired*() to restart the PID control with the new value.
     * @param kD The derivative value.
     * @return Itself
     */
    public TalonFXVelocityController withVoltagekD(double kD) {
        slot0_kD = kD;

        return this;
    }

    /**
     * Set the kS value for voltage PID control. After changing this value
     * call desired*() to restart the PID control with the new value.
     * @param kS The static feedforward voltage.
     * @return Itself
     */
    public TalonFXVelocityController withVoltagekS(double kS) {
        slot0_kS = kS;

        return this;
    }

    /**
     * Set the kV value for voltage PID control. After changing this value
     * call desired*() to restart the PID control with the new value.
     * @param kV The voltage required for 1 rps velocity.
     * @return Itself
     */
    public TalonFXVelocityController withVoltagekV(double kV) {
        slot0_kV = kV;

        return this;
    }

    /**
     * Set the kA value for voltage PID control. Only applies to Motion Magic.
     * After changing this value call desired*() to restart the PID control
     * with the new value.
     * @param kA The voltage for acceleration of 1 rps/s.
     * @return Itself
     */
    public TalonFXVelocityController withVoltagekA(double kA) {
        slot0_kA = kA;

        return this;
    }

    /**
     * Set the kP value for torque PID control. After changing this value
     * call desired*() to restart the PID control with the new value.
     * @param kP The proportional value. The amperage to apply for error of 1 rps.
     * 
     */
    public TalonFXVelocityController withTorquekP(double kP) {
        slot1_kP = kP;

        return this;
    }

    /**
     * Set the kI value for torque PID control. After changing this value
     * call desired*() to restart the PID control with the new value.
     * @param kI The integral value.
     * @return Itself
     */
    public TalonFXVelocityController withTorquekI(double kI) {
        slot1_kI = kI;

        return this;
    }

    /**
     * Set the kD value for torque PID control. After changing this value
     * call desired*() to restart the PID control with the new value.
     * @param kD The derivative value.
     * 
     */
    public TalonFXVelocityController withTorquekD(double kD) {
        slot1_kD = kD;

        return this;
    }

    /**
     * Set the kS value for torque PID control. After changing this value
     * call desired*() to restart the PID control with the new value.
     * @param kS The static feedforward voltage.
     * @return Itself
     */
    public TalonFXVelocityController withTorquekS(double kS) {
        slot1_kS = kS;

        return this;
    }

    /**
     * Returns the motor current velocity.
     * @return Velocity in rps.
     */
    public double getRPS() { return talon.getVelocity().getValueAsDouble(); }

    /**
     * Returns the motor current velocity.
     * @return Velocity in rpm.
     */
    public double getRPM() { return getRPS() * 60; }

    @Override
    public void initSendable( SendableBuilder builder )
    {
        builder.setSmartDashboardType("TalonFXVelocityController"); 
        builder.addBooleanProperty(".controllable", () -> true, null);

        builder.addDoubleProperty("Desired RPS", ()-> desiredRotationsPerSecond, null);            
        builder.addDoubleProperty("Motor RPS", this::getRPS, null);
        builder.addDoubleProperty("Motor RPM", this::getRPM, null);

        if (controlType == MotorControlType.voltage || controlType == MotorControlType.motionMagic) {
            builder.addDoubleProperty("Voltage kP", ()-> slot0_kP, this::withVoltagekP);
            builder.addDoubleProperty("Voltage kI", ()-> slot0_kI, this::withVoltagekI);
            builder.addDoubleProperty("Voltage kD", ()-> slot0_kD, this::withVoltagekD);
            builder.addDoubleProperty("Voltage kS", ()-> slot0_kS, this::withVoltagekS);
            builder.addDoubleProperty("Voltage kV", ()-> slot0_kV, this::withVoltagekV);
            builder.addDoubleProperty("Voltage kA", ()-> slot0_kA, this::withVoltagekA);
        } else {
            builder.addDoubleProperty("Torque kP", ()-> slot1_kP, this::withTorquekP);
            builder.addDoubleProperty("Torque kI", ()-> slot1_kI, this::withTorquekI);
            builder.addDoubleProperty("Torque kD", ()-> slot1_kD, this::withTorquekD);
            builder.addDoubleProperty("Torque kS", ()-> slot1_kS, this::withTorquekS);
        }
    }

    /**
     * Motor control types available. Note Motion Magic is configured the same
     * as voltage.
     */
    public enum MotorControlType {
        voltage,
        torque,
        motionMagic
        ;
    }
}
