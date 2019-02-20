package frc.robot.subsystems;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.utils.Util;
import frc.robot.utils.talon.TalonSRXChecker;
import frc.robot.utils.talon.TalonSRXFactory;
import frc.robot.utils.talon.TalonSRXUtil;

import java.util.ArrayList;

public class Elevator extends Subsystem {
    private static final double homePositionInches = 0; //TODO Change value
    private static final int slot = 0; //TODO Change name
    private static final int positionCrontrolSlot = 2;
    private static final int reverseSoftLimit = -1000000; //TODO Change
    private static final int forwardSoftLimit = 500; //TODO Change
    private static final double encoderTicksPerInch = 75.6; //TODO Change
    private static Elevator instance = null;
    private Intake intake = Intake.getInstance();
    private final TalonSRX master, slave;
    private PeriodicIO periodicIO = new PeriodicIO();
    private ElevatorControlState elevatorControlState = ElevatorControlState.OpenLoop;

    private boolean hasBeenZeroed = false;

    private Elevator(){
        master = TalonSRXFactory.createDefaultTalon(ElevatorConstants.elevatorMasterId); //TODO Change constant

        //TODO Change constants \/
        TalonSRXUtil.checkError(master.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, 0, 100), "Could not detect elevator encoder: "); //TODO Check input type
        TalonSRXUtil.checkError(master.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.longCANTimeoutMs), "Could not set forward (down) limit switch elevator: "); //TODO Change constants
        TalonSRXUtil.checkError(master.configForwardSoftLimitThreshold(forwardSoftLimit, Constants.longCANTimeoutMs), "Could not set forward (down) soft limit switch elevator: ");
        TalonSRXUtil.checkError(master.configForwardSoftLimitEnable(true, Constants.longCANTimeoutMs), "Could not enable forward (down) soft limit switch elevator: ");
        TalonSRXUtil.checkError(master.configVoltageCompSaturation(12, Constants.longCANTimeoutMs), "Could not set voltage compensation saturation elevator: ");
        TalonSRXUtil.checkError(master.configReverseSoftLimitThreshold(reverseSoftLimit, Constants.longCANTimeoutMs), "Could not set reverse (up) soft limit switch elevator: ");
        TalonSRXUtil.checkError(master.configReverseSoftLimitEnable(true, Constants.longCANTimeoutMs), "Could not enable reverse (up) soft limit switch elevator: ");
        TalonSRXUtil.checkError(master.config_kP(slot, ElevatorConstants.elevatorKP, Constants.longCANTimeoutMs), "Could not set elevator kp: ");
        TalonSRXUtil.checkError(master.config_kI(slot, ElevatorConstants.elevatorKI, Constants.longCANTimeoutMs), "Could not set elevator ki: ");
        TalonSRXUtil.checkError(master.config_kD(positionCrontrolSlot, ElevatorConstants.elevatorKD, Constants.longCANTimeoutMs), "Could not set elevator kd: ");
        TalonSRXUtil.checkError(master.config_kF(positionCrontrolSlot, ElevatorConstants.elevatorKF, Constants.longCANTimeoutMs), "Could not set elevator kf");
        TalonSRXUtil.checkError(master.configMaxIntegralAccumulator(slot, ElevatorConstants.maxIntegralAccumulator, Constants.longCANTimeoutMs), "Could not set elevator max integral: "); //TODO Change constants
        TalonSRXUtil.checkError(master.config_IntegralZone(slot, ElevatorConstants.iZone, Constants.longCANTimeoutMs), "Could not set elevator i zone: ");
        TalonSRXUtil.checkError(master.configAllowableClosedloopError(slot, ElevatorConstants.deadband, Constants.longCANTimeoutMs), "Could not set elevator deadband: ");
        TalonSRXUtil.checkError(master.configMotionAcceleration(ElevatorConstants.elevatorAcceleration, Constants.longCANTimeoutMs), "Could not set elevator acceleration ");
        TalonSRXUtil.checkError(master.configMotionCruiseVelocity(ElevatorConstants.elevatorVelocity, Constants.longCANTimeoutMs), "Could not set elevator cruise velocity: ");
        TalonSRXUtil.checkError(master.config_kP(positionCrontrolSlot, ElevatorConstants.elevatorJogKP, Constants.longCANTimeoutMs), "Could not set elevator kp: ");
        TalonSRXUtil.checkError(master.config_kI(positionCrontrolSlot, ElevatorConstants.elevatorJogKI, Constants.longCANTimeoutMs), "Could not set elevator ki: ");
        TalonSRXUtil.checkError(master.config_kD(positionCrontrolSlot, ElevatorConstants.elevatorJogKD, Constants.longCANTimeoutMs), "Could not set elevator kd: ");
        TalonSRXUtil.checkError(master.configMaxIntegralAccumulator(positionCrontrolSlot, ElevatorConstants.maxIntegralAccumulator, Constants.longCANTimeoutMs), "Could not set elevator max integral: ");
        TalonSRXUtil.checkError(master.config_IntegralZone(positionCrontrolSlot, ElevatorConstants.iZone, Constants.longCANTimeoutMs), "Could not set elevator i zone: ");
        TalonSRXUtil.checkError(master.configAllowableClosedloopError(positionCrontrolSlot, ElevatorConstants.deadband, Constants.longCANTimeoutMs), "Could not set elevator deadband: ");
        TalonSRXUtil.checkError(master.configClosedloopRamp(ElevatorConstants.elevatorRampRate, Constants.longCANTimeoutMs), "Could not set elevator voltage ramp rate: ");
        TalonSRXUtil.checkError(master.configOpenloopRamp(ElevatorConstants.elevatorRampRate, Constants.longCANTimeoutMs), "Could not set elevator voltage ramp rate: ");
        TalonSRXUtil.checkError(master.configContinuousCurrentLimit(20, Constants.longCANTimeoutMs), "Could not set elevator continuous current limit.");
        TalonSRXUtil.checkError(master.configPeakCurrentLimit(35, Constants.longCANTimeoutMs), "Could not set elevator peak current limit.");
        TalonSRXUtil.checkError(master.configPeakCurrentDuration(200, Constants.longCANTimeoutMs), "Could not set elevator peak current duration.");
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

        slave = TalonSRXFactory.createPermanentSlaveTalon(ElevatorConstants.elevatorSlaveId, ElevatorConstants.elevatorMasterId);
        slave.setInverted(true);

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
        double positionInchesFromHome = positionInchesOffGround - homePositionInches;
        double encoderPosition = positionInchesFromHome * encoderTicksPerInch;
        setClosedLoopRawPosition(encoderPosition);
    }
    
    public synchronized void setPositionPID(double positionInchesOffGround) {
        double positionInchesFromHome = positionInchesOffGround - homePositionInches;
        double encoderPosition = positionInchesFromHome * encoderTicksPerInch;
        if(elevatorControlState != ElevatorControlState.PositionPID) {
            elevatorControlState = ElevatorControlState.PositionPID;
            master.selectProfileSlot(positionCrontrolSlot, 0);
        }
        periodicIO.demand = encoderPosition;
    }
    
    public synchronized void setClosedLoopRawPosition(double encoderPosition){
        if(elevatorControlState != ElevatorControlState.MotionMagic) {
            elevatorControlState = ElevatorControlState.MotionMagic;
            master.selectProfileSlot(slot, 0);
        }
        periodicIO.demand = encoderPosition;
    }
    
    public synchronized boolean hasFinishedTrajectory() {
        return elevatorControlState == ElevatorControlState.MotionMagic && Util.epsilonEquals(periodicIO.activeTrajectoryPosition, periodicIO.demand, 6);
    }

    public synchronized double getRPM() {
        return periodicIO.velocityTicksPer100MS * 0; //TODO Modify constants
    }

    public synchronized double getInchesOffGround() {
        return (periodicIO.positionTicks / encoderTicksPerInch) + homePositionInches;
    }

    public synchronized double getSetpoint() {
        return elevatorControlState == ElevatorControlState.MotionMagic ? periodicIO.demand / encoderTicksPerInch + homePositionInches : Double.NaN;
    }

    public synchronized double getActiveTrajetoryAccelG(){
        return periodicIO.activeTrajectoryAccelG;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Elevator Output %", periodicIO.outputPercent);
        SmartDashboard.putNumber("Elevator RPM", getRPM());
        SmartDashboard.putNumber("Elevator Current", master.getOutputCurrent());
        SmartDashboard.putNumber("Elevator Height", getInchesOffGround());
        SmartDashboard.putBoolean("Elevator Limit", periodicIO.limitSwitch);
        SmartDashboard.putNumber("Elevator Sensor Height", periodicIO.positionTicks);

        SmartDashboard.putNumber("Elevator Last Expected Trajectory", periodicIO.demand);
        SmartDashboard.putNumber("Elevator Current Trajectory Point", periodicIO.activeTrajectoryPosition);
        SmartDashboard.putNumber("Elevator Traj Vel", periodicIO.activeTrajectoryVelocity);
        SmartDashboard.putNumber("Elevator Traj Accel", periodicIO.activeTrajectoryAccelG);
        SmartDashboard.putBoolean("Elevator Has Sent Trajectory", hasFinishedTrajectory());
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public synchronized void zeroSensors(){
        master.setSelectedSensorPosition(0, 0, 10);
        hasBeenZeroed = true;
    }

    public synchronized boolean hasBeenZeroed(){
        return hasBeenZeroed;
    }

    public synchronized void resetIfAtLimit(){
        if(periodicIO.limitSwitch){
            zeroSensors();
        }
    }

    private void setNeutralMode(NeutralMode neutralMode){
        master.setNeutralMode(neutralMode);
        slave.setNeutralMode(neutralMode);
    }

    @Override
    public synchronized void readPeriodicInputs(){
        final double t = Timer.getFPGATimestamp();
        periodicIO.positionTicks = master.getSelectedSensorPosition(0);
        periodicIO.velocityTicksPer100MS = master.getSelectedSensorVelocity(0);
        if(master.getControlMode() == ControlMode.MotionMagic){
            periodicIO.activeTrajectoryPosition = master.getActiveTrajectoryPosition();
            final int newVel = master.getActiveTrajectoryVelocity();
            if(Util.epsilonEquals(newVel, ElevatorConstants.elevatorVelocity, 5) || Util.epsilonEquals(newVel, periodicIO.activeTrajectoryVelocity, 5)){
                periodicIO.activeTrajectoryAccelG = 0.0;
            }else if(newVel > periodicIO.activeTrajectoryVelocity){
                periodicIO.activeTrajectoryAccelG = -ElevatorConstants.elevatorAcceleration; //TODO Modify num
            }else{
                periodicIO.activeTrajectoryAccelG = ElevatorConstants.elevatorAcceleration; //TODO Modify num
            }
            periodicIO.activeTrajectoryVelocity = newVel;
        }else{
            periodicIO.activeTrajectoryPosition = Integer.MIN_VALUE;
            periodicIO.activeTrajectoryVelocity = 0;
            periodicIO.activeTrajectoryAccelG = 0.0;
        }
        periodicIO.outputPercent = master.getMotorOutputPercent();
        periodicIO.limitSwitch = master.getSensorCollection().isFwdLimitSwitchClosed();
        periodicIO.t = t;

        if(getInchesOffGround() > ElevatorConstants.epsilon) {
            periodicIO.feedForward = intake.hasGamePiece() ? ElevatorConstants.feedForwardWithGamePiece : ElevatorConstants.feedForwardWithoutGamePiece; //TODO Change constants
        }else{
            periodicIO.feedForward = 0.0;
        }
    }

    @Override
    public synchronized void writePeriodicOutputs(){
        if(elevatorControlState == ElevatorControlState.MotionMagic){
            master.set(ControlMode.MotionMagic, periodicIO.demand, DemandType.ArbitraryFeedForward, periodicIO.feedForward);
        }else if(elevatorControlState == ElevatorControlState.PositionPID){
            master.set(ControlMode.Position, periodicIO.demand, DemandType.ArbitraryFeedForward, periodicIO.feedForward);
        }else{
            master.set(ControlMode.PercentOutput, periodicIO.demand, DemandType.ArbitraryFeedForward, periodicIO.feedForward);
        }
    }

    @Override
    @SuppressWarnings("Duplicates")
    public boolean checkSystem() {
        setNeutralMode(NeutralMode.Coast);

        boolean leftSide = TalonSRXChecker.CheckTalons(this, new ArrayList<TalonSRXChecker.TalonSRXConfig>(){
                {
                    add(new TalonSRXChecker.TalonSRXConfig("master", master));
                    add(new TalonSRXChecker.TalonSRXConfig("talon2", slave));
                }
            }, new TalonSRXChecker.CheckerConfig() {
                {
                    currentFloor = 2;
                    rpmFloor = 200;
                    currentEpsilon = 2.0;
                    rpmEpsilon = 250;
                    runTimeSec = 2.0;
                    runOutputPercentage = -0.4;
                    rpmSupplier = () -> master.getSelectedSensorVelocity(0);
                }
            });

        return leftSide;
    }

    public double getActiveTrajectorAccelG(){
        return 0.0; //TODO Implement
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
        public int activeTrajectoryPosition;
        public double outputPercent;
        public boolean limitSwitch;
        public double feedForward;
        public double t;

        //Outputs
        public double demand;
    }
}
