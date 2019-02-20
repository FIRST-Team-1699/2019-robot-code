package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import frc.robot.constants.DriveBaseConstants;
import frc.robot.utils.DriveSignal;
import frc.robot.utils.talon.TalonSRXFactory;

public class DriveBase extends Subsystem {

    private static DriveBase instance = null;
    private final TalonSRX portMaster, portSlave, starMaster, starSlave; 
    //private final DifferentialDrive drive;
    //private final SpeedControllerGroup portGroup, starGroup;
    private PeriodicIO periodicIO = new PeriodicIO();

    private DriveControlState mDriveControlState;
    

    private DriveBase() {
        portMaster = TalonSRXFactory.createDefaultTalon(DriveBaseConstants.starboardMasterPort);
        portSlave = TalonSRXFactory.createPermanentSlaveTalon(DriveBaseConstants.starboardSlavePort, DriveBaseConstants.starboardMasterPort);
        starMaster = TalonSRXFactory.createDefaultTalon(DriveBaseConstants.portMasterPort);
        starSlave = TalonSRXFactory.createPermanentSlaveTalon(DriveBaseConstants.portSlavePort,DriveBaseConstants.portMasterPort);
        starMaster.setInverted(true);
        starSlave.setInverted(true);

    }

    public static DriveBase getInstance() {
        if (instance == null) {
            instance = new DriveBase();
        }
        return instance;
    }

    public synchronized void setOpenLoop(DriveSignal signal) {
        if (mDriveControlState != mDriveControlState.OPEN_LOOP) {
            System.out.println("Switch to open loop");
            System.out.println(signal);
            mDriveControlState = mDriveControlState.OPEN_LOOP;
            portMaster.configNeutralDeadband(0.04,0);
            starMaster.configNeutralDeadband(0.04,0);
        }
        periodicIO.left_demand = signal.getLeft();
        periodicIO.right_demand = signal.getRight();
        periodicIO.left_feedforward = 0.0;
        periodicIO.right_feedforward = 0.0;
    }    

    @Override
    public synchronized void readPeriodicInputs() {

    }

    @Override
    public synchronized void writePeriodicOutputs() {
       starMaster.set(ControlMode.PercentOutput, periodicIO.left_demand, DemandType.ArbitraryFeedForward, 0.0);
       portMaster.set(ControlMode.PercentOutput, periodicIO.right_demand, DemandType.ArbitraryFeedForward, 0.0);
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

    public static class PeriodicIO {
        //Inputs
        public double outputPercent;
       


        //Outputs
        public double left_demand;
        public double right_demand;
        public double left_feedforward;
        public double right_feedforward;
    }

    public enum DriveControlState {
        OPEN_LOOP, // open loop voltage control
        PATH_FOLLOWING, // velocity PID control
    }

}