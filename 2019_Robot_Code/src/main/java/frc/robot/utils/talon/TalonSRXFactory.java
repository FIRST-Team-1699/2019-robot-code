package frc.robot.utils.talon;

import com.ctre.phoenix.ParamEnum;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class TalonSRXFactory {

    //TODO Check values
    private final static int timeoutMS = 100;

    public static class Configuration {
        public NeutralMode neutralMode = NeutralMode.Coast;
        public double neutralDeadband = 0.04;

        public boolean enableCurrentLimit = false;
        public boolean enableSoftLimit = false;
        public boolean enableLimitSwitch = false;
        public int forwardSoftLimit = 0;
        public int reverseSoftLimit = 0;

        public boolean inverted = false;
        public boolean sensorPhase = false;

        public int controlFramePeriodMS = 5;
        public int motionContolFramePeriodMS = 100;
        public int generalStatusFrameRateMS = 5;
        public int feedbackStatusFrameRateMS = 100;
        public int quadEncoderStatusFrameRateMS = 100;
        public int analogTempVBatStatusFrameRateMS = 100;
        public int pulseWidthStatusFrameRateMS = 100;

        public VelocityMeasPeriod velocityMeasurmentPeriod = VelocityMeasPeriod.Period_100Ms;
        public int velocityMeasurmentRollingAverageWindow = 64;

        public double openLoopRampRate = 0.0;
        public double closedLoopRampRate = 0.0;
    }

    public static final Configuration defaultConfiguration = new Configuration();
    public static final Configuration slaveConfiguration = new Configuration();

    static{
        slaveConfiguration.controlFramePeriodMS = 100;
        slaveConfiguration.motionContolFramePeriodMS = 1000;
        slaveConfiguration.generalStatusFrameRateMS = 1000;
        slaveConfiguration.feedbackStatusFrameRateMS = 1000;
        slaveConfiguration.quadEncoderStatusFrameRateMS = 1000;
        slaveConfiguration.analogTempVBatStatusFrameRateMS = 1000;
        slaveConfiguration.pulseWidthStatusFrameRateMS = 1000;
    }

    public static TalonSRX createDefaultTalon(final int id){
        return createTalon(id, defaultConfiguration);
    }

    public static TalonSRX createPermanentSlaveTalon(final int id, final int masterID){
        final TalonSRX talon = createTalon(id, slaveConfiguration);
        talon.set(ControlMode.Follower, masterID);
        return talon;
    }

    public static TalonSRX createTalon(final int id, final Configuration config){
        TalonSRX talon = new LazyTalonSRX(id);
        talon.set(ControlMode.PercentOutput, 0.0);

        talon.changeMotionControlFramePeriod(config.controlFramePeriodMS);
        talon.clearMotionProfileHasUnderrun(timeoutMS);
        talon.clearMotionProfileTrajectories();

        talon.clearStickyFaults(timeoutMS);

        talon.configForwardLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, timeoutMS);
        talon.configReverseLimitSwitchSource(LimitSwitchSource.FeedbackConnector, LimitSwitchNormal.NormallyOpen, timeoutMS);
        talon.overrideLimitSwitchesEnable(config.enableLimitSwitch);

        talon.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, timeoutMS);
        talon.configSetParameter(ParamEnum.eClearPositionOnLimitF, 0, 0, 0, timeoutMS);

        talon.configNominalOutputForward(0, timeoutMS);
        talon.configNominalOutputForward(0, timeoutMS);
        talon.configNeutralDeadband(config.neutralDeadband, timeoutMS);

        talon.configPeakOutputForward(1.0, timeoutMS);
        talon.configPeakOutputReverse(-1.0, timeoutMS);

        talon.setNeutralMode(config.neutralMode);

        talon.configForwardSoftLimitThreshold(config.forwardSoftLimit, timeoutMS);
        talon.configForwardSoftLimitEnable(config.enableSoftLimit, timeoutMS);

        talon.configReverseSoftLimitThreshold(config.reverseSoftLimit, timeoutMS);
        talon.configReverseSoftLimitEnable(config.enableSoftLimit, timeoutMS);
        talon.overrideLimitSwitchesEnable(config.enableSoftLimit);

        talon.setInverted(config.inverted);
        talon.setSensorPhase(config.sensorPhase);

        talon.selectProfileSlot(0, 0);

        talon.configVelocityMeasurementPeriod(config.velocityMeasurmentPeriod, timeoutMS);
        talon.configVelocityMeasurementWindow(config.velocityMeasurmentRollingAverageWindow, timeoutMS);

        talon.configOpenloopRamp(config.openLoopRampRate, timeoutMS);
        talon.configClosedloopRamp(config.closedLoopRampRate, timeoutMS);

        talon.configVoltageCompSaturation(0.0, timeoutMS);
        talon.configVoltageMeasurementFilter(32, timeoutMS);
        talon.enableVoltageCompensation(false);

        talon.enableCurrentLimit(config.enableCurrentLimit);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, config.quadEncoderStatusFrameRateMS, timeoutMS);
        talon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, config.feedbackStatusFrameRateMS, timeoutMS);

        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, config.quadEncoderStatusFrameRateMS, timeoutMS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, config.analogTempVBatStatusFrameRateMS, timeoutMS);
        talon.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, config.pulseWidthStatusFrameRateMS, timeoutMS);

        talon.setControlFramePeriod(ControlFrame.Control_3_General, config.controlFramePeriodMS);

        return talon;
    }
}
