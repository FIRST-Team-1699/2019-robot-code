package frc.robot.utils;

import edu.wpi.first.wpilibj.Timer;

public class TimeDelayedBoolean {
    private Timer t = new Timer();
    private boolean old = false;

    public boolean update(boolean value, double timeout){
        if(!old && value){
            t.reset();
            t.start();
        }
        old = value;
        return value && t.get() >= timeout;
    }
}
