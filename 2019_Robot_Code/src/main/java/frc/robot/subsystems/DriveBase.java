package frc.robot.subsystems;

import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveBase extends Subsystem {

    private static DriveBase instance = null;
    private final VictorSP portMaster, portSlave, starMaster, starSlave; //TODO Change to talons
    private final DifferentialDrive drive;
    private final SpeedControllerGroup portGroup, starGroup;
    private PeriodicIO periodicIO = new PeriodicIO();

    private DriveBase(){
        portMaster = new VictorSP(0);
        portSlave = new VictorSP(1);
        starMaster = new VictorSP(4);
        starSlave = new VictorSP(5);

        portGroup = new SpeedControllerGroup(portMaster, portSlave);
        starGroup = new SpeedControllerGroup(starMaster, starSlave);

        drive = new DifferentialDrive(portGroup, starGroup);
    }

    public static DriveBase getInstance(){
        if(instance == null){
            instance = new DriveBase();
        }
        return instance;
    }

    @Override
    public synchronized void readPeriodicInputs() {

    }

    @Override
    public synchronized void writePeriodicOutputs(){
        drive.arcadeDrive(periodicIO.rotateDemand, periodicIO.forwardDemand);
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

    public synchronized void setOpenLoop(double forwardDemand, double rotateDemand){
        //TODO Change state
        periodicIO.forwardDemand = forwardDemand;
        periodicIO.rotateDemand = rotateDemand;
    }

    public static class PeriodicIO {
        //Inputs
        public double outputPercent;

        //Outputs
        public double rotateDemand;
        public double forwardDemand;
    }
}
