package Team4450.Robot26.utility;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import java.util.List;

public class LinkedMotors {
    private final TalonFX masterMotor;
    private final List<TalonFX> slaveMotors;
    
    public LinkedMotors(TalonFX master, TalonFX... slaves) {
        this.masterMotor = master;
        this.slaveMotors = List.of(slaves);
    }

    public void set(double power) {
        this.masterMotor.set(power);
        for (int i = 0; i < this.slaveMotors.size(); i++) {
            this.slaveMotors.get(i).set(power);
        }
    }

    public void applyConfiguration(TalonFXConfiguration configuration) {
        this.masterMotor.getConfigurator().apply(configuration);
        for (int i = 0; i < this.slaveMotors.size(); i++) {
            this.slaveMotors.get(i).getConfigurator().apply(configuration);
        }
    }
}
