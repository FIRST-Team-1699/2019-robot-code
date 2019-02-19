package frc.robot.subsystems;

import frc.robot.loops.ILooper;

public abstract class Subsystem {

    public void writeToLog(){

    }

    public void readPeriodicInputs(){

    }

    public void writePeriodicOutputs(){

    }

    public abstract boolean checkSystem();

    public abstract void outputTelemetry();

    public abstract void stop();

    public void zeroSensors(){

    }

    public void registerEnabledLoops(ILooper enabledLooper){

    }

}