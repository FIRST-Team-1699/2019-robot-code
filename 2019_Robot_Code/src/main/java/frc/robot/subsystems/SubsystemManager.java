//Credit for code to FRC Team 254
//https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/frc2018/SubsystemManager.java

package frc.robot.subsystems;

import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.loops.Looper;

import java.util.ArrayList;
import java.util.List;

public class SubsystemManager implements ILooper {

    private final List<Subsystem> subsystems;
    private List<Loop> loops = new ArrayList<>();

    public SubsystemManager(List<Subsystem> subsystems){
        this.subsystems = subsystems;
    }

    public void outputToDashboard() {
        subsystems.forEach(Subsystem::outputTelemetry);
    }

    public void writeToLog() {
        subsystems.forEach(Subsystem::writeToLog);
    }

    public void stop() {
        subsystems.forEach(Subsystem::stop);
    }

    private class EnabledLoop implements Loop {

        @Override
        public void onStart(double timestamp) {
            for(Loop loop : loops){
                loop.onStart(timestamp);
            }
        }

        @Override
        public void onLoop(double timestamp) {
            for(Subsystem subsystem : subsystems){
                subsystem.readPeriodicInputs();
            }
            for(Loop loop : loops){
                loop.onLoop(timestamp);
            }
            for(Subsystem subsystem : subsystems){
                subsystem.writePeriodicOutputs();
            }
        }

        @Override
        public void onStop(double timestamp) {
            for(Loop loop : loops){
                loop.onStop(timestamp);
            }
        }
    }

    private class DisabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {

        }

        @Override
        public void onLoop(double timestamp) {
            for(Subsystem subsystem : subsystems){
                subsystem.readPeriodicInputs();
            }
            for(Subsystem subsystem : subsystems){
                subsystem.writePeriodicOutputs();
            }
        }

        @Override
        public void onStop(double timestamp) {

        }
    }

    public void registerEnabledLoops(Looper enabledLooper){
        subsystems.forEach(s -> s.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    @Override
    public void register(Loop loop) {
        loops.add(loop);
    }
}
