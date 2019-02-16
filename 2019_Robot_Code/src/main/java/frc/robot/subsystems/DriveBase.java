package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.constants.DriveBaseConstants;

public class DriveBase extends Subsystem {

    private static DriveBase instance = null;
    private final SpeedController portMaster, portSlave, starMaster, starSlave; 
    private final DifferentialDrive drive;
    private final SpeedControllerGroup portGroup, starGroup;
    private PeriodicIO periodicIO = new PeriodicIO();

    private DriveBase() {
        portMaster = new WPI_TalonSRX(DriveBaseConstants.starboardMasterPort);
        portSlave = new WPI_TalonSRX(DriveBaseConstants.starboardSlavePort);
        starMaster = new WPI_TalonSRX(DriveBaseConstants.portMasterPort);
        starSlave = new WPI_TalonSRX(DriveBaseConstants.portSlavePort);

        portGroup = new SpeedControllerGroup(portMaster, portSlave);
        starGroup = new SpeedControllerGroup(starMaster, starSlave);

        drive = new DifferentialDrive(portGroup, starGroup);
    }

    public static DriveBase getInstance() {
        if (instance == null) {
            instance = new DriveBase();
        }
        return instance;
    }

    @Override
    public synchronized void readPeriodicInputs() {

    }

    @Override
    public synchronized void writePeriodicOutputs() {
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

    public synchronized void setOpenLoop(double forwardDemand, double rotateDemand) {
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

