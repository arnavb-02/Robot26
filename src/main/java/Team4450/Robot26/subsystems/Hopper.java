package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.InvertedValue;

import com.ctre.phoenix6.CANBus;

public class Hopper extends SubsystemBase {
    private final TalonFX hopperMotor = new TalonFX(Constants.HOPPER_MOTOR_CAN_ID, new CANBus(Constants.CANIVORE_NAME));

    public Hopper() {
        // Configure motor neutral mode
        hopperMotor.setNeutralMode(NeutralModeValue.Coast);

        TalonFXConfiguration hopperCFG = new TalonFXConfiguration();

        // Neutral + inversion
        hopperCFG.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        hopperCFG.CurrentLimits = new CurrentLimitsConfigs().withSupplyCurrentLimit(Constants.HOPPER_CURRENT_LIMIT);
        hopperCFG.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;

        this.hopperMotor.getConfigurator().apply(hopperCFG);

        SmartDashboard.putNumber("Hopper Power", Constants.HOPPER_MOTOR_POWER);
        SmartDashboard.putNumber("Hopper Current", getHooperCurrent());

        hopperMotor.set(0);
    }

    public void start() {
        double power = SmartDashboard.getNumber("Hopper Power", Constants.HOPPER_MOTOR_POWER);
        hopperMotor.set(power);
    }

    public void startSlow() {
        hopperMotor.set(0.2);
    }

    public void stop() {
        hopperMotor.set(0);
    }

    public double getHooperCurrent() {
        return hopperMotor.getSupplyCurrent(true).getValueAsDouble();
    }
}
