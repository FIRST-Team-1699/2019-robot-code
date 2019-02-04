package frc.robot.statemachine;

import frc.robot.states.IntakeState;
import frc.robot.utils.TimeDelayedBoolean;

//TODO Change to account for hatch
public class IntakeStateMachine {
    public static final double actuationTime = 0.0;
    public static final double exchangeShootSetpoint = 1.0;
    public static final double shootBallSetpoint = 0.5;
    private static final double intakeBallSetpoint = 0.5;
    //TODO Add more speed setpoints
    public static final double holdSetpoint = 0.0;
    public static final double lostGamePieceTime = 0.25;
    public static final double unclampWaitingTime = 1.0;

    public enum WantedAction {
        WantManual,
        WantGamePiece
    }

    private enum SystemState {
        OpenLoop,
        KeepingGamePiece
    }

    private SystemState systemState = SystemState.OpenLoop;
    private IntakeState commandedState = new IntakeState();
    private double currentStateStartTime = 0;

    private TimeDelayedBoolean lastSeenGamePiece = new TimeDelayedBoolean();
    private double lastSeenGamePieceTime = Double.NaN;

    private IntakeState.JawState wantedJawState = IntakeState.JawState.Closed;
    private double wantedPower = 0.0;
    private boolean forceClose = false;

    public synchronized void setWantedJawState(final IntakeState.JawState jawState){
        wantedJawState = jawState;
    }

    public synchronized void forceJawClosed(boolean closed) {
        forceClose = closed;
    }

    public synchronized void setWantedPower(double power){
        wantedPower = power;
    }

    public IntakeState update(double timestamp, WantedAction wantedAction, IntakeState currentState) {
        synchronized (IntakeStateMachine.this){
            SystemState newState;
            double timeInState = timestamp - currentStateStartTime;

            switch(systemState){
                case OpenLoop:
                    newState = handleOpenLoopTransitions(wantedAction, currentState);
                    break;
                case KeepingGamePiece:
                    newState = handleKeepingGamePieceTransitions(wantedAction, currentState);
                    break;
                default:
                    System.out.println("Unexprected intake system state: " + systemState);
                    newState = systemState;
                    break;
            }

            if(newState != systemState){
                System.out.println(timestamp + ": Intake changed state: " + systemState + " -> " + newState);
                systemState = newState;
                currentStateStartTime = timestamp;
                lastSeenGamePiece.update(false, lostGamePieceTime);
            }

            switch (systemState){
                case OpenLoop:
                    getOpenLoopCommandedState(currentState, commandedState);
                    break;
                case KeepingGamePiece:
                    getKeepingGamePieceCommandedState(currentState, commandedState, timestamp);
                    break;
                default:
                    getOpenLoopCommandedState(currentState, commandedState);
                    break;
            }
        }
        return commandedState;
    }

    private synchronized SystemState handleOpenLoopTransitions(WantedAction wantedAction, IntakeState currentState){
        switch(wantedAction){
            case WantGamePiece:
                lastSeenGamePieceTime = Double.NaN;
                return SystemState.KeepingGamePiece;
            default:
                return SystemState.OpenLoop;
        }
    }

    private synchronized void getOpenLoopCommandedState(IntakeState currentState, IntakeState commandedState){
        commandedState.setPower(wantedPower);
        if(mustStayClosed(currentState)){
            commandedState.jawState = IntakeState.JawState.Closed;
        }else{
            commandedState.jawState = wantedJawState;
        }
        //TODO Add LEDs?
    }

    private synchronized void getKeepingGamePieceCommandedState(IntakeState currentState, IntakeState commandedState, double timestamp){
        commandedState.setPower(intakeBallSetpoint);
        boolean close = (currentState.seesBall() && wantedJawState != IntakeState.JawState.Open) || mustStayClosed(currentState);

        boolean currentlySeeBall = currentState.seesBall();
        boolean resetSeenBallTime = true;
        if(!currentlySeeBall && !Double.isNaN(lastSeenGamePieceTime) && (timestamp - lastSeenGamePieceTime < lostGamePieceTime)) {
            currentlySeeBall = true;
            close = (wantedJawState != IntakeState.JawState.Open) || mustStayClosed(currentState);
            resetSeenBallTime = false;
        }

        boolean seenBall = lastSeenGamePiece.update(currentlySeeBall, lostGamePieceTime);

        if(currentlySeeBall) {
            if(!seenBall) {
                commandedState.setPower(intakeBallSetpoint);
            }else{
                commandedState.setPower(holdSetpoint);
            }
            commandedState.jawState = close ? IntakeState.JawState.Closed : IntakeState.JawState.Open;

            if(resetSeenBallTime){
                lastSeenGamePieceTime = timestamp;
            }
        }else{
            commandedState.setPower(intakeBallSetpoint);
            if(forceClose) {
                commandedState.jawState = IntakeState.JawState.Closed;
            }else if(!Double.isNaN(lastSeenGamePieceTime) && (timestamp - lastSeenGamePieceTime < unclampWaitingTime)){
                commandedState.jawState = IntakeState.JawState.Closed;
            }else{
                commandedState.jawState = mustStayClosed(currentState) ? IntakeState.JawState.Closed : IntakeState.JawState.Open;
            }
        }
    }

    private boolean mustStayClosed(IntakeState state) {
        return state.wristSetpoint < 0 || state.wristAngle < 0; //TODO Change constants
    }
}
