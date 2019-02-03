package frc.robot.utils.talon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class LazyTalonSRX extends TalonSRX {
    protected double lastSet = Double.NaN;
    protected ControlMode lastControlMode = null;

    public LazyTalonSRX(final int deviceNumber){
        super(deviceNumber);
    }

    public double getLastSet(){
        return lastSet;
    }

    @Override
    public void set(ControlMode mode, double value){
        if(value != lastSet || mode != lastControlMode) {
            lastSet = value;
            lastControlMode = mode;
            super.set(mode, value);
        }
    }
}
