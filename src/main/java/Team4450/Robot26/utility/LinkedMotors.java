package Team4450.Robot26.utility;

import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import java.util.List;

public class LinkedMotors {
    private final TalonFX masterMotor;
    private final List<TalonFX> slaveMotors;
    
    public LinkedMotors(TalonFX master, TalonFX... slaves) {
        this.masterMotor = master;
        this.slaveMotors = List.of(slaves);
    }

    @Deprecated
    public void set(double power) {
        this.masterMotor.set(power);
        for (int i = 0; i < slaveMotors.size(); i++) {
            this.slaveMotors.get(i).set(power);
        }
    }

    @Deprecated
    public void setControl(ControlRequest req, boolean shooter) {
        this.masterMotor.setControl(req);
        if (shooter) {
            this.slaveMotors.get(0).setControl(new Follower(this.masterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
            this.slaveMotors.get(1).setControl(new Follower(this.masterMotor.getDeviceID(), MotorAlignmentValue.Aligned));
            this.slaveMotors.get(2).setControl(new Follower(this.masterMotor.getDeviceID(), MotorAlignmentValue.Opposed));
        }
    }

    @Deprecated
    public TalonFX getMotor(int i) {
        if (i < 0 || i + 1 > this.slaveMotors.size()) {
            return null;
        }
        if (i == 0) {
            return this.masterMotor;
        } else {
            return this.slaveMotors.get(i - 1);
        }
    }
    
    @Deprecated
    public int size() {
        return this.slaveMotors.size() + 1;
    }
}
