package frc.robot.statemachine;

import java.util.HashMap;
import java.util.Map;

public class ElevatorClawFSM { //TODO Is this/should this actually be a FSM?

    //TODO Make a singleton
    //TODO Figure out how to make elevator read state from this class or how to make this class create action in the elevator

    public enum State{ //TODO Check accessibility modifier
        HatchLow, //Set elevator and claw to low hatch position
        HatchMiddle, //Set elevator and claw to middle hatch position
        HatchHigh, //Set elevator and claw to high hatch position
        BallLow, //Set elevator and claw to low ball position
        BallCargo, //Set elevator and claw to score ball in cargo ship position
        BallMiddle, //Set elevator and claw to middle ball position
        BallHigh, //Set elevator and claw to high ball position
        BallPickup //Set elevator and claw to ball pickup position
    }

    public enum Button{ //TODO Add more buttons

    }

    private Map<Button, State> actionMap; //Maps button to corresponding action

    public ElevatorClawFSM(){
        actionMap = new HashMap<>(); //TODO Check map type
        //TODO Populate map
    }

    public void setState(Button button){
        //TODO Populate with code to change state
    }
}
