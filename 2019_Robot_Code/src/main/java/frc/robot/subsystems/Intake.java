//Credit for code to FRC Team 254
//https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/frc2018/subsystems/Intake.java

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.ClawConstants;
import frc.robot.constants.PnumaticsConstants;
import frc.robot.loops.ILooper;
import frc.robot.loops.Loop;
import frc.robot.statemachine.IntakeStateMachine;
import frc.robot.states.IntakeState;

public class Intake extends Subsystem {
    private static final boolean closed = false;
    private static final boolean extended = true;

    private static Intake instance;
    private final DoubleSolenoid grabberSolenoid; //TODO Add comment for open/close
    private final VictorSP portMaster;
    private final VictorSP starMaster;
    private final CarriageCanifier canifier = CarriageCanifier.getInstance();
    //TODO Add led control?
    private IntakeStateMachine.WantedAction wantedAction = IntakeStateMachine.WantedAction.WantManual;
    private IntakeState.JawState jawState;
    private IntakeState currentState = new IntakeState();
    private IntakeStateMachine intakeStateMachine = new IntakeStateMachine();
    private boolean clawOpen;

    private Intake(){
        grabberSolenoid = new DoubleSolenoid(PnumaticsConstants.pcmid, PnumaticsConstants.ClawPistonsOpen, PnumaticsConstants.ClawPistonsClosed);
        grabberSolenoid.set(Value.kReverse);
        clawOpen = false;
        portMaster = new VictorSP(ClawConstants.portMaster);
        starMaster = new VictorSP(ClawConstants.starMaster);
        portMaster.set(0);
        starMaster.set(0);
        portMaster.setInverted(true);
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
    public void registerEnabledLoops(ILooper enabledLooper) {
        Loop loop = new Loop() {

            @Override
            public void onStart(double timestamp) {
                if(hasGamePiece()) {
                    wantedAction = IntakeStateMachine.WantedAction.WantGamePiece; //TODO add multiple game pieces?
                }else{
                    wantedAction = IntakeStateMachine.WantedAction.WantManual;
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
        portMaster.set(state.leftMotor);
        starMaster.set(state.rightMotor);
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
        return true;
    }
    
    public void toggleClawOpen(){
        //System.out.println("Toggle");
        if(grabberSolenoid.get() == Value.kReverse){
            grabberSolenoid.set(Value.kForward);
            clawOpen = true;
        }else if(grabberSolenoid.get() == Value.kForward){
            clawOpen = false;
            grabberSolenoid.set(Value.kReverse);
        }

    }
    
}