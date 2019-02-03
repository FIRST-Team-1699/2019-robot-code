package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Talon;
import frc.robot.utils.talon.TalonSRXFactory;
import frc.robot.utils.talon.TalonSRXUtil;

public class Elevator extends Subsystem {
    public static final double homePositionInches = 0; //TODO Change value
    private static final int slot = 0; //TODO Change name
    private static final int positionFrontrolSlot = 2;
    private static final int reverseSoftLimit = -1000000; //TODO Change
    private static final int forwardSoftLimit = 500; //TODO Change
    private static final double encoderTicksPerInch = 0; //TODO Change
    private static Elevator instance = null;
    private Intake intake = Intake.getInstance();
    private final TalonSRX master, talon2, talon3, talon4;
    private PeriodicIO periodicIO = new PeriodicIO();
    private ElevatorControlState elevatorControlState = ElevatorControlState.OpenLoop;

    private boolean hasBeenZeroed = false;

    private Elevator(){
        master = TalonSRXFactory.createDefaultTalon(0); //TODO Change constant

        //TODO Change constants \/
        TalonSRXUtil.checkError(master.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, 100), "Could not detect elevator encoder: "); //TODO Check input type
        TalonSRXUtil.checkError(master.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, 0), "Could not set forward (down) limit switch elevator: "); //TODO Change constants
        TalonSRXUtil.checkError(master.configForwardSoftLimitThreshold(forwardSoftLimit, 0), "Could not set forward (down) soft limit switch elevator: ");
        TalonSRXUtil.checkError(master.configForwardSoftLimitEnable(true, 0), "Could not enable forward (down) soft limit switch elevator: ");
        TalonSRXUtil.checkError(master.configVoltageCompSaturation(12, 0), "Could not set voltage compensation saturation elevator: ");
        TalonSRXUtil.checkError(master.configReverseSoftLimitThreshold(reverseSoftLimit, 0), "Could not set reverse (up) soft limit switch elevator: ");
        TalonSRXUtil.checkError(master.configReverseSoftLimitEnable(true, 0), "Could not enable reverse (up) soft limit switch elevator: ");
        TalonSRXUtil.checkError(master.config_kP(slot, 0, 0), "Could not set elevator kp: ");
        TalonSRXUtil.checkError(master.config_kI(slot, 0, 0), "Could not set elevator ki: ");
        TalonSRXUtil.checkError(master.configMaxIntegralAccumulator(slot, 0, 0), "Could not set elevator max integral: "); //TODO Change constants
        TalonSRXUtil.checkError(master.config_IntegralZone(slot, 0, 0), "Could not set elevator i zone: ");
        TalonSRXUtil.checkError(master.configAllowableClosedloopError(slot, 0, 0), "Could not set elevator deadband: ");
        TalonSRXUtil.checkError(master.configMotionAcceleration(0, 0), "Could not set elevator acceleration ");
        TalonSRXUtil.checkError(master.configMotionCruiseVelocity(0, 0), "Could not set elevator cruise velocity: ");
        TalonSRXUtil.checkError(master.config_kP(positionFrontrolSlot, 0, 0), "Could not set elevator kp: ");
        TalonSRXUtil.checkError(master.config_kI(positionFrontrolSlot, 0, 0), "Could not set elevator ki: ");
        TalonSRXUtil.checkError(master.config_kD(positionFrontrolSlot, 0, 0), "Could not set elevator kd: ");
        TalonSRXUtil.checkError(master.configMaxIntegralAccumulator(positionFrontrolSlot, 0, 0), "Could not set elevator max integral: ");
        TalonSRXUtil.checkError(master.config_IntegralZone(positionFrontrolSlot, 0, 0), "Could not set elevator i zone: ");
        TalonSRXUtil.checkError(master.configAllowableClosedloopError(positionFrontrolSlot, 0, 0), "Could not set elevator deadband: ");
        TalonSRXUtil.checkError(master.configClosedloopRamp(0, 0), "Could not set elevator voltage ramp rate: ");
        TalonSRXUtil.checkError(master.configOpenloopRamp(0, 0), "Could not set elevator voltage ramp rate: ");
        TalonSRXUtil.checkError(master.configContinuousCurrentLimit(20, 0), "Could not set elevator continuous current limit.");
        TalonSRXUtil.checkError(master.configPeakCurrentLimit(35, 0), "Could not set elevator peak current limit.");
        TalonSRXUtil.checkError(master.configPeakCurrentDuration(200, 0), "Could not set elevator peak current duration.");
        master.enableCurrentLimit(true);

        master.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
        master.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);

        master.selectProfileSlot(0, 0);

        master.overrideLimitSwitchesEnable(true);
        master.overrideSoftLimitsEnable(false);

        master.enableVoltageCompensation(true);

        master.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
        master.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);

        master.setInverted(true); //TODO Check
        master.setSensorPhase(true);

        //TODO Add slaves
        talon2 = new TalonSRX(0);
        talon3 = new TalonSRX(0);
        talon4 = new TalonSRX(0);

        master.set(ControlMode.PercentOutput, 0);
        setNeutralMode(NeutralMode.Brake);
    }

    public synchronized static Elevator getInstance() {
        if(instance == null){
            instance = new Elevator();
        }
        return instance;
    }

    public synchronized void setOpenLoop(double percentage){
        elevatorControlState = ElevatorControlState.OpenLoop;
        periodicIO.demand = percentage;
    }

    public synchronized void setMotionMagicPosition(double positionInchesOffGround) {

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

    private void setNeutralMode(NeutralMode neutralMode){
        //TODO Implement
    }

    private enum ElevatorControlState {
        OpenLoop,
        MotionMagic,
        PositionPID
    }

    public static class PeriodicIO {
        //Inputs
        public int positionTicks;
        public int velocityTicksPer100MS;
        public double activeTrajectoryAccelG;
        public int activeTrajectoryVelocity;
        public int activeTrajectoruPosition;
        public double outputPercent;
        public boolean limitSwitch;
        public double feedForward;
        public double t;

        //Outputs
        public double demand;
    }
}
