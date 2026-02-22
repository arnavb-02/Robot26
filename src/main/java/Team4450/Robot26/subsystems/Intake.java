package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import com.ctre.phoenix6.CANBus;

public class Intake extends SubsystemBase {

    // This motor is a Kraken x60
    private final TalonFX intakePivitMotor = new TalonFX(Constants.INTAKE_MOTOR_PIVIT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x44
    private final TalonFX intakeMotorLeft = new TalonFX(Constants.INTAKE_MOTOR_LEFT_CAN_ID);
    // This motor is a Kraken x44
    private final TalonFX intakeMotorRight = new TalonFX(Constants.INTAKE_MOTOR_RIGHT_CAN_ID);

    private boolean canPivit;
    private boolean canSpin;

    // This value is expected to be between 0 and 1
    private double pivitTargetPosition;
    // The format of this value is in rotations of the pivit motor
    private double pivitTargetPositionMotorPosition;

    // This value is expected to be between 0 and 1
    private double pivitCurrentPosition;
    // The format of this value is in rotations of the pivit motor
    private double pivitCurrentPositionMotorPosition;

    private boolean runIntake;

    public Intake() {
        this.canPivit = intakePivitMotor.isConnected();
        this.canSpin = intakeMotorLeft.isConnected() && intakeMotorRight.isConnected();

        // Assume the pivit starting position is 0
        this.pivitCurrentPosition = 0;
        this.pivitTargetPosition = 0;

        if (this.canPivit) {
            this.intakePivitMotor.setPosition(0);
        }

        TalonFXConfiguration intakeCFG = new TalonFXConfiguration();

        // Neutral + inversion
        intakeCFG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        intakeCFG.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.SHOOTER_INFEED_CURRENT_LIMIT);
        intakeCFG.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        this.intakeMotorLeft.getConfigurator().apply(intakeCFG);
        this.intakeMotorRight.getConfigurator().apply(intakeCFG);

        SmartDashboard.putBoolean("Intake can Pivit", canPivit);
        SmartDashboard.putBoolean("Intake can Spin", canSpin);
        SmartDashboard.putNumber("Pivit Position", 0);

        SmartDashboard.putNumber("Intake Target RPM", Constants.INTAKE_DEFAULT_TARGET_RPM);
    }

    @Override
    public void periodic() {
        this.pivitTargetPosition = SmartDashboard.getNumber("Pivit Position", 0);
        if (this.canPivit) {
            this.pivitTargetPositionMotorPosition = this.pivitPositionToMotorPosition(this.pivitTargetPosition);
            SmartDashboard.putNumber("ppmp", this.pivitTargetPositionMotorPosition);
            // Convert position input to rotations for the motor
            double power = Constants.INTAKE_PIVIT_MOTOR_POWER;
            
            if (this.pivitCurrentPositionMotorPosition <= this.pivitTargetPositionMotorPosition - Constants.INTAKE_PIVIT_TOLERENCE_MOTOR_ROTATIONS) {
                this.intakePivitMotor.set(power);
            } else if (this.pivitCurrentPositionMotorPosition >= this.pivitTargetPositionMotorPosition + Constants.INTAKE_PIVIT_TOLERENCE_MOTOR_ROTATIONS) {
                this.intakePivitMotor.set(-power);
            } else {
                this.intakePivitMotor.set(0);
            }

            this.pivitCurrentPositionMotorPosition = this.getPivitPosition();
            this.pivitCurrentPosition = this.motorPositionToPivitPosition(this.pivitCurrentPositionMotorPosition);
            SmartDashboard.putNumber("Pivit current position", this.pivitCurrentPosition);

            SmartDashboard.putNumber("Intake RPM", getIntakeRPM());

            if (this.runIntake) {
                setIntakeRPM(SmartDashboard.getNumber("Intake Target RPM", Constants.INTAKE_DEFAULT_TARGET_RPM));
            }

            SmartDashboard.putNumber("Intake Current Draw", getIntakeCurrent());
        }
    }

    // Linear interpolate the pivit position between zero and one with the motor rotations of up and down on the pivit
    public double pivitPositionToMotorPosition(double pivitPosition) {
        return Constants.INTAKE_PIVIT_MOTOR_POSITION_UP + ((Constants.INTAKE_PIVIT_MOTOR_POSITION_DOWN - Constants.INTAKE_PIVIT_MOTOR_POSITION_UP) * pivitPosition);
    }

    public double motorPositionToPivitPosition(double motorPosition) {
        return (motorPosition / Constants.INTAKE_PIVIT_GEAR_RATIO) * (1 / (Constants.INTAKE_PIVIT_POSITION_DOWN_DEGREES / 360));
    }

    public void startIntake() {
        if (canSpin) {
            this.runIntake = true;
        }
    }

    public void testIntake() {
        if (canSpin) {
            this.intakeMotorLeft.set(0.05);
            this.intakeMotorRight.setControl(new Follower(this.intakeMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    // TODO: FIX for the start Intake command for autos
    public void startIntakeSlow() {
        if (canSpin) {
            // intakeMotors.setPower(0.1);
        }
    }

    public void stopIntake() {
        if (canSpin) {
            this.runIntake = false;
            this.intakeMotorLeft.set(0);
            this.intakeMotorRight.setControl(new Follower(this.intakeMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    public double getIntakeRPM() {
        if (canSpin) {
            return intakeMotorLeft.getRotorVelocity(true).getValueAsDouble() * 60;
        } else {
            return -1;
        }
    }

    public double getIntakeCurrent() {
        return intakeMotorLeft.getSupplyCurrent(true).getValueAsDouble() + intakeMotorRight.getSupplyCurrent(true).getValueAsDouble();
    }

    public double getIntakeLeftMotorCurrent() {
        if (canSpin) {
            return intakeMotorLeft.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeRightMotorCurrent() {
        if (canSpin) {
            return intakeMotorRight.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeVoltage() {
        if (canSpin) {
            return intakeMotorLeft.getSupplyVoltage(true).getValueAsDouble() + intakeMotorRight.getSupplyVoltage(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeLeftMotorVoltage() {
        if (canSpin) {
            return intakeMotorLeft.getSupplyVoltage(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeRightMotorVoltage() {
        if (canSpin) {
            return intakeMotorRight.getSupplyVoltage(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public void setPivitMotorSpeed(double speed) {
        if (canPivit) {
            intakePivitMotor.set(speed);
        }
    }

    // The position input is between 0 and 1 with 0 being up and 1 being down
    public void setPivitMotorPosition(double position) {
        pivitTargetPosition = position;
    }

    // TODO:
    public double getPivitPosition() {
        // Need to convert
        if (canPivit) {
            return intakePivitMotor.getPosition(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getPivitMotorCurrent() {
        if (canPivit) {
            return intakePivitMotor.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getPivitMotorVoltage() {
        if (canPivit) {
            return intakePivitMotor.getSupplyVoltage(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public boolean hasDevices() {
        return intakePivitMotor.isConnected() && intakeMotorLeft.isConnected() && intakeMotorRight.isConnected();
    }

    /**
     * Sets the power for the intake motors.
     * @param power The power level to set.
     */
    public void setPower(double power) {
        this.intakePivitMotor.set(power);
    }

    public void setIntakeRPM(double targetRPM) {
        double currentRPM = getIntakeRPM();
        double error = targetRPM - currentRPM;
        double adjustment = Constants.INTAKE_kP * error; // Adjustment to approach target
        double newRPM = targetRPM + adjustment; // Adjust current RPM towards target
        this.intakeMotorLeft.set(newRPM / Constants.INTAKE_MAX_THEORETICAL_RPM);
        this.intakeMotorRight.setControl(new Follower(this.intakeMotorLeft.getDeviceID(), MotorAlignmentValue.Opposed));
    }
}
