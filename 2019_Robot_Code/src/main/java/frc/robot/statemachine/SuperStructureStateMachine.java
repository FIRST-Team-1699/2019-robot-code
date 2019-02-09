package frc.robot.statemachine;

//TODO Give proper credit
//Code inspiration from https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/frc2018/statemachines/SuperstructureStateMachine.java

import frc.robot.states.SuperstructureCommand;

public class SuperStructureStateMachine {
    public enum Action{
        Idle,
        MoveToPosition,
        SwitchToManual
    }
    //TODO make states for each position not just a generic position
    public enum SystemState{
        HoldPosition,
        MoveingToPosition,
        Manual
    }

    private SystemState currentSystemState = SystemState.HoldPosition;
    private SuperstructureCommand command = new SuperstructureCommand();
    private SuperstructureCommand currentCommandState = new SuperstructureCommand();
    private SuperstructureCommand desiredCommandState = new SuperstructureCommand();

    private String motionPlan = new String ("Place Holder"); //TODO Change to actual motion plan

    private double elevatorHeight = 0; //TODO Change to stow height
    private double clawAngle = 0; //TODO Change to stow claw angle

    private double openLoopPower = 0.0;
    private double maxHeight = 0.0; //TODO Change

    public synchronized void resetManual(){
        openLoopPower = 0.0;
    }

    public synchronized void setMaxHeight(final double height){
        maxHeight = height;
    }

    public synchronized void setOpenLoopPower(final double power){
        openLoopPower = power;
    }

    public synchronized void setElevatorHeight(final double inches){
        elevatorHeight = inches;
    }

    public synchronized double getElevatorHeight(){
        return elevatorHeight;
    }

    public synchronized void setClawAngle(final double angle){
        clawAngle = angle;
    }

    public synchronized double getClawAngle(){
        return clawAngle;
    }

    public synchronized void jogElevator(final double relativeInches){
        elevatorHeight += relativeInches;
        elevatorHeight = Math.min(elevatorHeight, maxHeight);
        elevatorHeight = Math.max(elevatorHeight, 0); //TODO Change to max height
    }

    public synchronized void jogWrite(final double relativeDegrees){
        clawAngle += relativeDegrees;
        clawAngle = Math.min(clawAngle, 0); //TODO Change to min angle
        clawAngle = Math.max(clawAngle, 0); //TODO Change to max angle
    }

    public synchronized boolean scoringPosChanged(){
        return false; //TODO Implement
    }

    public synchronized SystemState getSystemState(){
        return currentSystemState;
    }

    public synchronized void setUpwardSubcommandEnabled(boolean enabled){
        //TODO Implement
    }

    //TODO Add update method
}
