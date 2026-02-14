package Team4450.Robot26.subsystems;

import Team4450.Robot26.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import java.util.ResourceBundle.Control;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.controls.TorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

public class Hopper extends SubsystemBase {
    private final TalonFX hopperMotor = new TalonFX(Constants.HOPPER_MOTOR_CAN_ID, new CANBus(Constants.CANIVORE_NAME));

    public Hopper() {
        // Configure motor neutral mode
        hopperMotor.setNeutralMode(NeutralModeValue.Coast);

        SmartDashboard.putNumber("Hopper Power", Constants.HOPPER_MOTOR_POWER);
        SmartDashboard.putNumber("Hopper Current", 0);

        hopperMotor.set(0);
    }

    public void start() {
        double power = SmartDashboard.getNumber("Hopper Power", Constants.HOPPER_MOTOR_POWER);
        // double current = SmartDashboard.getNumber("Hopper Current", 0);
        hopperMotor.set(power);
    }

    public void stop() {
        hopperMotor.set(0);
    }
}
