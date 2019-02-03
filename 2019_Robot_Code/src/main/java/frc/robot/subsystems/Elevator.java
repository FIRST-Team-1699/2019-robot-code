package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class Elevator extends Subsystem {
    public static final double homePositionInches = 0; //TODO Change value
    //TODO Add gearing?
    private static final int positionFrontrolSlot = 2;
    private static final int reverseSoftLimit = -1000000; //TODO Change
    private static final int forwardSoftLimit = 500; //TODO Change
    private static final double encoderTicksPerInch = 0; //TODO Change
    private static Elevator instance = null;
    private Intake intake = Intake.getInstance();
    private final TalonSRX talon1, talon2, talon3, talon4;
    private PeriodicIO periodicIO = PeriodicIO();
    private ElevatorControlState elevatorControlState = ElevatorControlState.OpenLoop;

    private boolean hasBeenZeroed = false;

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
