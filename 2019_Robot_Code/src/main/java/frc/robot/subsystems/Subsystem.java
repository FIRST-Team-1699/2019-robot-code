//Credit for code to FRC Team 254
//https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/frc2018/subsystems/Subsystem.java

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