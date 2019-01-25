package frc.robot.statemachine;

//TODO Give proper credit
//Code inspiration from https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/frc2018/statemachines/SuperstructureStateMachine.java

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

    private SystemState currectSystemState = SystemState.HoldPosition;
    private String currentCommandState = new String("Place Holder"); //TODO Change to actual command
    private String desiredCommandState = new String("Place Holder"); //TODO Change to actual command

    private String motionPlan = new String ("Place Holder"); //TODO Change to actual motion plan

    private double elevatorHeight = 0; //TODO Change to stow height
    private double clawAngle = 0; //TODO Change to stow claw angle

}
