package frc.robot.utils.crashtracking;

public abstract class CrashTrackingRunnable implements Runnable {

    @Override
    public void run() {
        try{
            runCrashTracked();
        }catch (Throwable t){
            CrashTracker.logThrowableCrash(t);
            throw t;
        }
    }

    public abstract void runCrashTracked();
}
