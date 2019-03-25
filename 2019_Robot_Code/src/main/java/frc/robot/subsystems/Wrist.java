package frc.robot.subsystems;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.Constants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.utils.Util;
import frc.robot.utils.crashtracking.ReflectingCSVWriter;
import frc.robot.utils.talon.TalonSRXChecker;
import frc.robot.utils.talon.TalonSRXFactory;
import frc.robot.utils.talon.TalonSRXUtil;

import java.util.ArrayList;

public class Wrist extends Subsystem {
    private static final int magicMotionSlot = 0; //TODO Change all constants
    private static final int positionControlSlot = 1;
    private static final int forwardSoftLimit = 0;
    private static final int reverseSoftLimit = 0;

    private static final double homingOutput = -0.25;
    private boolean hasBeenZeroed = false;

    private static Wrist instance;
    private final Intake intake = Intake.getInstance();
    private final CarriageCanifier canifier = CarriageCanifier.getInstance();
    private final Elevator elevator = Elevator.getInstance();
    private final TalonSRX master;
    private final PeriodicIO periodicIO = new PeriodicIO();
    private double zeroPosition = Double.NaN;
    private SystemState systemState = SystemState.Homing; //TODO Check init state
    private SystemState desiredState = SystemState.MotionProfiling; //TODO Check init state
    private ReflectingCSVWriter<PeriodicIO> csvWriter = null;

    private Wrist(){
        master = TalonSRXFactory.createDefaultTalon(ClawConstants.wristMoter);
        //TODO Add sensor control
        ErrorCode errorCode;

        /* errorCode = master.configRemoteFeedbackFilter(0, RemoteSensorSource.CANifier_Quadrature, 0, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist encoder!!!: " + errorCode, false);
        } */

        errorCode = master.configSelectedFeedbackSensor(FeedbackDevice.Analog, 0, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Count not detect wrist encoder: " + errorCode, false);
        }

        /* errorCode = master.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set forward limit switch wrist: " + errorCode, false);
        } */

       /* errorCode = master.configReverseLimitSwitchSource(RemoteLimitSwitchSource.RemoteCANifier, LimitSwitchNormal.NormallyOpen, canifier.getDeviceID(), Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set reverse limit switch wrist: " + errorCode, false);
        }*/

        errorCode = master.configForwardSoftLimitThreshold(forwardSoftLimit, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set forward soft limit switch wrist: " + errorCode, false);
        }

        errorCode = master.configForwardSoftLimitEnable(true, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not enable forward soft limit switch wrist: " + errorCode, false);
        }

        errorCode = master.configReverseSoftLimitThreshold(reverseSoftLimit, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set reverse soft limit switch wrist: " + errorCode, false);
        }

        errorCode = master.configReverseSoftLimitEnable(true, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not enable reverse soft limit switch wrist: " + errorCode, false);
        }

        errorCode = master.configVoltageCompSaturation(12.0, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist voltage compensation: " + errorCode, false);
        }

        errorCode = master.config_kP(magicMotionSlot, ClawConstants.wristKp, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist kp: " + errorCode, false);
        }

        errorCode = master.config_kI(magicMotionSlot, ClawConstants.wristKi, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist ki: " + errorCode, false);
        }

        errorCode = master.config_kD(magicMotionSlot, ClawConstants.wristKd, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist kd: " + errorCode, false);
        }

        errorCode = master.config_kF(magicMotionSlot, ClawConstants.wristKf, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist ki: " + errorCode, false);
        }

        errorCode = master.configMaxIntegralAccumulator(magicMotionSlot, ClawConstants.wristMaxIntegralAccumulator, Constants.longCANTimeoutMs);
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist max integral: " + errorCode, false);
        }

        errorCode = master.config_IntegralZone(magicMotionSlot, ClawConstants.wristIZone, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist i zone: " + errorCode, false);
        }

        errorCode = master.configAllowableClosedloopError(magicMotionSlot, ClawConstants.wristDeadband, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist deadband: " + errorCode, false);
        }

        errorCode = master.configMotionAcceleration(ClawConstants.wristAcceleration, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist acceleration: " + errorCode, false);
        }

        errorCode = master.configMotionCruiseVelocity(ClawConstants.wristCruiseVelocity, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist cruise velocity: " + errorCode, false); //TODO Change constants
        }

        errorCode = master.config_kP(positionControlSlot, ClawConstants.wristJogKd, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist kp: " + errorCode, false);
        }

        errorCode = master.config_kI(positionControlSlot, ClawConstants.wristKi, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist ki: " + errorCode, false);
        }

        errorCode = master.config_kD(positionControlSlot, ClawConstants.wristJogKd, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist kd: " + errorCode, false);
        }

        errorCode = master.configMaxIntegralAccumulator(positionControlSlot, ClawConstants.wristMaxIntegralAccumulator, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist max integral: " + errorCode, false);
        }

        errorCode = master.config_IntegralZone(positionControlSlot, ClawConstants.wristIZone, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist i zone: " + errorCode, false);
        }

        errorCode = master.configAllowableClosedloopError(positionControlSlot, ClawConstants.wristDeadband, Constants.longCANTimeoutMs); //TODO Change constants
        if(errorCode != ErrorCode.OK){
            DriverStation.reportError("Could not set wrist deadband: " + errorCode, false);
        }

        TalonSRXUtil.checkError(master.configContinuousCurrentLimit(20, Constants.longCANTimeoutMs), "Could not set wrist continuous current limit."); //TODO Change constants
        TalonSRXUtil.checkError(master.configPeakCurrentLimit(40, Constants.longCANTimeoutMs), "Could not set wrist peak current limit."); //TODO Change constants
        TalonSRXUtil.checkError(master.configPeakCurrentDuration(200, Constants.longCANTimeoutMs), "Could not set wrist peak current duration."); //TODO Change constants
        TalonSRXUtil.checkError(master.configClosedloopRamp(0, Constants.longCANTimeoutMs), "Could not set wrist voltage ramp rate: "); //TODO Change constants

        master.enableCurrentLimit(true);
        master.selectProfileSlot(0, 0); //TODO Check values
        master.setInverted(false);
        master.setSensorPhase(false);
        master.setNeutralMode(NeutralMode.Brake);
        master.overrideLimitSwitchesEnable(true);
        master.overrideSoftLimitsEnable(true);
        master.enableVoltageCompensation(true);
        master.set(ControlMode.PercentOutput, 0);
        master.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 10, 20);
        master.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10, 20);
        master.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, 0);
        master.configSetParameter(ParamEnum.eClearPositionOnLimitR, 0, 0, 0, 0);
    }

    public synchronized static Wrist getInstance() {
        if(instance == null){
            instance = new Wrist();
        }
        return instance;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putNumber("Wrist Angle", getAngle());
        SmartDashboard.putNumber("Wrist Position", getPosition());
        SmartDashboard.putNumber("Wrist Ticks", periodicIO.positionTicks);
        SmartDashboard.putNumber("Wrist periodic demand", periodicIO.demand);
        SmartDashboard.putBoolean("LIMR", periodicIO.limitSwitch);

        SmartDashboard.putNumber("Wrist RPM", getRPM());
        SmartDashboard.putNumber("Wrist Power %", periodicIO.outputPercent);
        SmartDashboard.putBoolean("Wrist Limit Switch", periodicIO.limitSwitch);
        SmartDashboard.putNumber("Wrist Last Expected Trajectory", getSetpoint());
        SmartDashboard.putNumber("Wrist Current Trajectory Point", periodicIO.activeTrajectoryPosition);
        SmartDashboard.putNumber("Wrist Traj Vel", periodicIO.activeTrajectoryVelocity);
        SmartDashboard.putNumber("Wrist Traj Accel", periodicIO.activeTrajectoryAccelerationRadPerS2);
        SmartDashboard.putBoolean("Wrist Has Sent Trajectory", hasFinishedTrajectory());
        SmartDashboard.putNumber("Wrist feedforward", periodicIO.feedForward);

        if(csvWriter != null){
            csvWriter.write();
        }
    }

    public synchronized void setRampRate(final double rampRate){
        master.configClosedloopRamp(rampRate, 0);
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public synchronized void zeroSensors(){
        master.setSelectedSensorPosition(0, 0, 0);
        canifier.resetWristEncoder();
        hasBeenZeroed = true;
    }

    public synchronized boolean hasBeenZeroed(){
        return hasBeenZeroed;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper){
        enabledLooper.register(new Loop() {
            double homingStartTime;
            boolean homeFound;

            @Override
            public void onStart(double timestamp) {
                homingStartTime = timestamp;
                //TODO Add logging?
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Wrist.this) {
                    if(!Double.isNaN(zeroPosition) && desiredState != systemState){
                        System.out.println(timestamp + ": Wrist changed states: " + systemState + " ->" + desiredState);
                        systemState = desiredState;
                    }

                    switch (systemState){
                        case OpenLoop:
                            break;
                        case MotionProfiling:
                            break;
                        case Homing:
                            //TODO Broken in 254?
                            systemState = SystemState.OpenLoop;
                            break;
                        default:
                           // System.out.println("Fell through on Wrist states!");
                    }
                }
            }

            @Override
            public void onStop(double timestamp) {
                //TODO Add logging?
            }
        });
    }

    public synchronized void setOpenLoop(double percentage){
        periodicIO.demand = percentage;
        desiredState = SystemState.OpenLoop;
    }

    public synchronized boolean resetIfAtLimit(){
        if(canifier.getLimR()){
            zeroSensors();
            return true;
        }
        return false;
    }

    public void setClosedLoop(int position){
        periodicIO.demand = position;
        desiredState = SystemState.MotionProfiling;
    }

    public synchronized void setMotionProfileAngle(double angle){
        periodicIO.demand = degreesToSensorUnits(angle);
        if(desiredState != SystemState.MotionProfiling) {
            desiredState = SystemState.MotionProfiling;
            master.selectProfileSlot(magicMotionSlot, 0);
        }
    }

    public synchronized void setPositionPIDAngle(double angle){
        periodicIO.demand = degreesToSensorUnits(angle);
        if(desiredState != SystemState.PositionPID) {
            desiredState = SystemState.PositionPID;
            master.selectProfileSlot(positionControlSlot, 0);
        }
    }

    public synchronized double getPosition(){
        return periodicIO.positionTicks;
    }

    public synchronized double getAngle(){
        return sensorUnitsToDegrees(periodicIO.positionTicks);
    }

    public double getRPM(){
        return sensorUnitsToDegrees(periodicIO.velocityTicksPer100ms);
    }

    public double getDegreesPerSecond(){
        return sensorUnitsToDegrees(periodicIO.velocityTicksPer100ms) * 10; //TODO Check math
    }

    public synchronized boolean hasFinishedTrajectory(){
        if(Util.epsilonEquals(periodicIO.activeTrajectoryPosition, degreesToSensorUnits(getSetpoint()), 2)){
            return true;
        }
        return false;
    }

    public synchronized double getSetpoint(){
        return desiredState == SystemState.MotionProfiling || desiredState == SystemState.PositionPID ? sensorUnitsToDegrees(periodicIO.demand) : Double.NaN;
    }

    private double sensorUnitsToDegrees(double units){
        return units; //TODO Add math
    }

    private double degreesToSensorUnits(double degrees){
        return degrees; //TODO Add math
    }

    @Override
    public synchronized void readPeriodicInputs(){
        if(master.hasResetOccurred()){
            DriverStation.reportError("Wrist Talon Reset!", false);
        }
        StickyFaults faults = new StickyFaults();
        master.getStickyFaults(faults);
        if(faults.hasAnyFault()){
            //DriverStation.reportError("Wrist Talon Fault" + faults.toString(), false);
            master.clearStickyFaults(0);
        }
        if(master.getControlMode() == ControlMode.MotionMagic){
            periodicIO.activeTrajectoryPosition = master.getActiveTrajectoryPosition();

            if(periodicIO.activeTrajectoryPosition < reverseSoftLimit){
                DriverStation.reportError("Active trajectory past reverse soft limit!", false);
            }else if(periodicIO.activeTrajectoryPosition > forwardSoftLimit) {
                DriverStation.reportError("Active trajectory past forward soft limit!", false);
            }
            final int newVel = master.getActiveTrajectoryVelocity();

            if(Util.epsilonEquals(newVel, 0, 5) || Util.epsilonEquals(newVel, periodicIO.activeTrajectoryVelocity, 5)){ //TODO Check constants
                periodicIO.activeTrajectoryAccelerationRadPerS2 = 0.0;
            }else{
                periodicIO.activeTrajectoryAccelerationRadPerS2 = Math.signum(newVel - periodicIO.activeTrajectoryVelocity) * 0 * 20.0 * Math.PI / 4096; //TODO Check constants
            }
            periodicIO.activeTrajectoryVelocity = newVel;
        }else{
            periodicIO.activeTrajectoryPosition = Integer.MIN_VALUE;
            periodicIO.activeTrajectoryVelocity = 0;
            periodicIO.activeTrajectoryAccelerationRadPerS2 = 0.0;
        }
        periodicIO.limitSwitch = canifier.getLimR();
        periodicIO.outputVoltage = master.getMotorOutputVoltage();
        periodicIO.outputPercent = master.getMotorOutputPercent();
        periodicIO.positionTicks = master.getSelectedSensorPosition(0);
        periodicIO.velocityTicksPer100ms = master.getSelectedSensorVelocity(0);
        
        if(getAngle() > 0 || sensorUnitsToDegrees(periodicIO.activeTrajectoryPosition) > 0) { //TODO Change constants
            double wristGravityComponent = Math.cos(Math.toRadians(getAngle())) * (intake.hasGamePiece() ? 0 : 0); //TODO Change constants
           double elevatorAccelerationComponent = elevator.getActiveTrajectorAccelG() * 0; //TODO Change constants
            double wristAccelerationComponent = periodicIO.activeTrajectoryAccelerationRadPerS2 * (intake.hasGamePiece() ? 0 : 0); //TODO Change constants
           periodicIO.feedForward = elevatorAccelerationComponent * wristGravityComponent + wristAccelerationComponent;
        }else{
            if(getSetpoint() < Util.epsilon) {
                periodicIO.feedForward = -0.1;
            }else{
                periodicIO.feedForward = 0.1;
            }
        }

        if(csvWriter != null){
            csvWriter.add(periodicIO);
        }

    }
    
    @Override
    public synchronized void writePeriodicOutputs() {
        if(desiredState == SystemState.MotionProfiling){
            master.set(ControlMode.MotionMagic, periodicIO.demand, DemandType.ArbitraryFeedForward, periodicIO.feedForward);
        }else if(desiredState == SystemState.PositionPID) {
            master.set(ControlMode.Position, periodicIO.demand, DemandType.ArbitraryFeedForward, periodicIO.feedForward);
        }else{
            master.set(ControlMode.PercentOutput, periodicIO.demand, DemandType.ArbitraryFeedForward, periodicIO.feedForward);
        }
    }

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.CheckTalons(this, new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
            {
                add(new TalonSRXChecker.TalonSRXConfig("wristMaster", master));
            }
        }, new TalonSRXChecker.CheckerConfig(){
            {
                runTimeSec = 1.0;
                runOutputPercentage = 0.20;
                
                rpmFloor = 50.0;
                currentFloor = 2.0;
                
                rpmSupplier = () -> master.getSelectedSensorVelocity(0);
            }
        });
    }

    public enum SystemState{
        Homing,
        MotionProfiling,
        PositionPID,
        OpenLoop
    }

    public static class PeriodicIO {
        public int positionTicks;
        public int velocityTicksPer100ms;
        public int activeTrajectoryPosition;
        public int activeTrajectoryVelocity;
        public double activeTrajectoryAccelerationRadPerS2;
        public double outputPercent;
        public double outputVoltage;
        public double feedForward;
        public boolean limitSwitch;

        public double demand;
    }
}
