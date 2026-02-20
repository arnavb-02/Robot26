package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import Team4450.Robot26.utility.LinkedMotors;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix6.hardware.TalonFX;

import com.ctre.phoenix6.CANBus;

public class Intake extends SubsystemBase {

    // This motor is a Kraken x60
    private final TalonFX intakePivitMotor = new TalonFX(Constants.INTAKE_MOTOR_PIVIT_CAN_ID, new CANBus(Constants.CANIVORE_NAME));
    // This motor is a Kraken x44
    private final TalonFX intakeLeftMotor = new TalonFX(Constants.INTAKE_MOTOR_LEFT_CAN_ID);
    // This motor is a Kraken x44
    private final TalonFX intakeRightMotor = new TalonFX(Constants.INTAKE_MOTOR_RIGHT_CAN_ID);
    private final LinkedMotors intakeMotors = new LinkedMotors(intakeLeftMotor, intakeRightMotor);

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

    private double intakeTargetRPM;

    public Intake() {
        this.canPivit = intakePivitMotor.isConnected();
        this.canSpin = intakeLeftMotor.isConnected() && intakeRightMotor.isConnected();

        // Assume the pivit starting position is 0
        this.pivitCurrentPosition = 0;
        this.pivitTargetPosition = 0;

        if (this.canPivit) {
            this.intakePivitMotor.setPosition(0);
        }

        SmartDashboard.putBoolean("Intake can Pivit", canPivit);
        SmartDashboard.putBoolean("Intake can Spin", canSpin);
        SmartDashboard.putNumber("Pivit Position", 0);
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
            intakeMotors.setPower(0.5); // Updated to use setPower
        }
    }

    public void startIntakeWithSpeed(double speed) {
        if (canSpin) {
            intakeMotors.setPower(speed); // Updated to use setPower
        }
    }

    public void stopIntake() {
        if (canSpin) {
            intakeMotors.setPower(0); // Updated to use setPower
        }
    }

    public double getIntakeRPM() {
        if (canSpin) {
            return intakeLeftMotor.getRotorVelocity(true).getValueAsDouble() * 60;
        } else {
            return -1;
        }
    }

    public double getIntakeCurrent() {
        if (canSpin) {
            return intakeLeftMotor.getSupplyCurrent(true).getValueAsDouble() + intakeRightMotor.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeLeftMotorCurrent() {
        if (canSpin) {
            return intakeLeftMotor.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeRightMotorCurrent() {
        if (canSpin) {
            return intakeRightMotor.getSupplyCurrent(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeVoltage() {
        if (canSpin) {
            return intakeLeftMotor.getSupplyVoltage(true).getValueAsDouble() + intakeRightMotor.getSupplyVoltage(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeLeftMotorVoltage() {
        if (canSpin) {
            return intakeLeftMotor.getSupplyVoltage(true).getValueAsDouble();
        } else {
            return -1;
        }
    }

    public double getIntakeRightMotorVoltage() {
        if (canSpin) {
            return intakeRightMotor.getSupplyVoltage(true).getValueAsDouble();
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

    /**
     * Sets the power for the intake motors.
     * @param power The power level to set.
     */
    public void setPower(double power) {
        this.intakePivitMotor.set(power);
    }

    public void setIntakeRPM(double targetRPM) {
        this.intakeTargetRPM = targetRPM;
        double currentRPM = intakeLeftMotor.getRotorVelocity(true).getValueAsDouble() * 60.0;
        double error = targetRPM - currentRPM;
        double adjustment = Constants.INTAKE_kP * error; // Adjustment to approach target
        double newRPM = currentRPM + adjustment; // Adjust current RPM towards target
        intakeMotors.setPower(newRPM / Constants.INTAKE_MAX_THEORETICAL_RPM); // Normalize to motor power
    }
}
