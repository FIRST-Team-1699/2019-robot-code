package frc.robot.statemachine;

import frc.robot.states.IntakeState;
import frc.robot.utils.TimeDelayedBoolean;

public class IntakeStateMachine {
    public static final double actuationTime = 0.0;
    public static final double exchangeShootSetpoint = 1.0;
    public static final double shootBallSetpoint = 0.5;
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

    //TODO Line 95
}
