package frc.robot.subsystems;

import frc.robot.loops.ILooper;

public abstract class Subsystem {

    public void writeToLog(){

    }

    public void readPeriodicInput(){

    }

    public void writePeriodicOutput(){

    }

    public abstract void checkSystem();

    public abstract void outputTelemetry();

    public abstract void stop();

    public void zeroSensors(){

    }

    public void registerEnabledLooper(ILooper enabledLooper){ //TODO Change to looper

    }
}
