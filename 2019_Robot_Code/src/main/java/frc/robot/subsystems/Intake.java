package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.states.IntakeState;
import frc.robot.utils.talon.TalonSRXChecker;
import frc.robot.utils.talon.TalonSRXFactory;

import java.util.ArrayList;

public class Intake extends Subsystem {
    private static final boolean closed = false;
    private static final boolean extended = true;

    private static Intake instance;
    private final Solenoid closeSolenoid, extendSolenoid; //TODO Add comment for open/close
    private final TalonSRX leftMaster, rightMaster;
    private final CarriageCanifier canifier = CarriageCanifier.getInstance();
    //TODO Add led control?
    private IntakeStateMachine.WantedAction wantedAction = IntakeStateMachine.WantedAction.WantManual;
    private IntakeState.JawState jawState;
    private IntakeState currentState = new IntakeState();
    private IntakeStateMachine intakeStateMachine = new IntakeStateMachine();

    private Intake(){
        closeSolenoid = new Solenoid(0); //TODO Add make solenoid in constants and add id
        extendSolenoid = new Solenoid(0); //TODO Add make solenoid in constants and add id

        leftMaster = TalonSRXFactory.createDefaultTalon(0); //TODO Add id
        leftMaster.set(ControlMode.PercentOutput, 0);
        leftMaster.setInverted(true);
        leftMaster.configVoltageCompSaturation(12.0, 0); //TODO Set constants \/
        leftMaster.enableVoltageCompensation(true);

        rightMaster = TalonSRXFactory.createDefaultTalon(0) //TODO Add id
        rightMaster.set(ControlMode.PercentOutput, 0);
        rightMaster.setInverted(false);
        rightMaster.configVoltageCompSaturation(12.0, 0); //TODO Set constants
        rightMaster.enableVoltageCompensation(true);
    }

    public static Intake getInstance(){
        if(instance == null){
            instance = new Intake();
        }
        return instance;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("Left Banner", getLeftBannerSensor());
        SmartDashboard.putBoolean("Right Banner", getRightBannerSensor());
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors(){
    }

    private IntakeState getCurrentState(){
        currentState.leftSensorTriggered = getLeftBannerSensor();
        currentState.rightSensorTriggered = getRightBannerSensor();
        currentState.wristAngle = Wrist.getInstance().getAngle();
        currentState.wristSetpoint = Wrist.getInstance().getSetpoint();
        return currentState;
    }

    @Override
    public void registedEnabledLoops(ILooper enabledLooper) {
        Loop loop = new Loop() {

            @Override
            public void onStart(double timestamp) {
                if(hasGamePiece()) {
                    wantedAction = IntakeStateMachine.WantGamePiece; //TODO add multiple game pieces?
                }else{
                    wantedAction = IntakeStateMachine.WantManual;
                }
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Intake.this) {
                    IntakeState newState = intakeStateMachine.update(Timer.getFPGATimestamp(), wantedAction, getCurrentState());
                    updateActuatorFromState(newState);
                }
            }

            @Override
            public void onStop(double timestamp) {
                wantedAction = IntakeStateMachine.WantedAction.WantManual;
                stop();
            }
        };
        enabledLooper.register(loop);
    }

    private void setJaw(IntakeState.JawState state){
        //TODO Implement
    }

    private synchronized void updateActuatorFromState(IntakeState state){
        leftMaster.set(ControlMode.PercentOutput, state.leftMotor);
        rightMaster.set(ControlMode.PercentOutput, state.rightMotor);
        setJaw(state.jawState);

        //TODO Add LEDs?
    }

    public synchronized boolean hasGamePiece(){
        return false; //TODO Implement
    }

    private boolean getLeftBannerSensor() {
        return canifier.getLeftBannerSensor();
    }
    private boolean getRightBannerSensor(){
        return canifier.getRightBannerSensor();
    }

    public IntakeState.JawState getJawState(){
        return jawState;
    }

    public synchronized void setState(IntakeStateMachine.WantedAction wantedAction){
        this.wantedAction = wantedAction;
    }

    public synchronized void setPower(double power){
        intakeStateMachine.setWantedPower(power);
    }

    public synchronized void shootBall(double power){
        setState(IntakeStateMachine.WantedAction.WantManual);
        setPower(power);
    }

    public IntakeStateMachine.WantedAction getWantedAction(){
        return wantedAction;
    }

    public synchronized void getOrKeepGamePiece() {
        setState(IntakeStateMachine.WantedAction.WantGamePiece);
    }

    public synchronized void tryOpenJaw() {
        intakeStateMachine.setWantedJawState(IntakeState.JawState.Open);
    }

    //TODO Add rest of jaw/grabber states

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.CheckTalons(this, new ArrayList<TalonSRXChecker.TalonSRXConfig>() {
            {
                add(new TalonSRXChecker.TalonSRXConfig("intake right master", rightMaster));
                add(new TalonSRXChecker.TalonSRXConfig("intake left master", leftMaster));
            }
        }, new TalonSRXChecker.CheckerConfig() {
            {
                currentFloor = 2;
                currentEpsilon = 2.0;
                rpmSupplier = null;
            }
        });
    }
}