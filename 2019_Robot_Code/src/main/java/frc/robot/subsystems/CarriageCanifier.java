package frc.robot.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;

public class CarriageCanifier extends Subsystem {
    private static CarriageCanifier instance;
    private CANifier canifier;
    private PeriodicInputs periodicInputs;
    private PeriodicOutputs periodicOutputs;
    private boolean outputsChanged = true;

    private CarriageCanifier() {
        canifier = new CANifier(0); //TODO Change constant
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 100, 0); //TODO Change constant
        canifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, 0); //TODO Change constant
        periodicInputs = new PeriodicInputs();
        periodicOutputs = new PeriodicOutputs();

        outputsChanged = true;
    }

    public static CarriageCanifier getInstance() {
        if(instance == null) {
            instance = new CarriageCanifier();
        }
        return instance;
    }

    public int getWristTicks(){
        return canifier.getQuadraturePosition();
    }

    public synchronized boolean getLeftBannerSensor() {
        return periodicInputs.leftSensorState;
    }

    public synchronized boolean getRightBannerSensor() {
        return periodicInputs.rightSensorState;
    }

    public synchronized boolean getLimR() {
        return periodicInputs.limr;
    }

    public synchronized void resetWristEncoder() {
        canifier.setQuadraturePosition(0, 0);
    }

    public int getDeviceId() {
        return canifier.getDeviceID();
    }

    //TODO Add LEDs?

    @Override
    public synchronized void readPeriodicInputs(){
        CANifier.PinValues pins = new CANifier.PinValues();
        canifier.getGeneralInputs(pins);
        periodicInputs.leftSensorState = pins.SCL;
        periodicInputs.rightSensorState = pins.SDA;
        periodicInputs.limr = !pins.LIMR;
    }

    //TODO Add Write for LEDs

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {
        periodicOutputs = new PeriodicOutputs();
        outputsChanged = true;
        writePeriodicOutputs();
    }

    @Override
    public void zeroSensors() {
        periodicInputs = new PeriodicInputs();
    }

    public static class PeriodicInputs {
        public boolean leftSensorState;
        public boolean rightSensorState;
        public boolean limr;
    }

    public static class PeriodicOutputs {
        //TODO Add LED colors?
    }
}
