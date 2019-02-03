package frc.robot.subsystems;

public class Intake extends Subsystem {

    private static Intake instance;

    public static Intake getInstance(){
        if(instance == null){
            instance = new Intake();
        }
        return instance;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }
}
