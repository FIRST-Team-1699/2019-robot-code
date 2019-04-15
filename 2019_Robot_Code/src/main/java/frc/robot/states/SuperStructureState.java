//Credit for code to FRC Team 254
//https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/frc2018/states/SuperstructureState.java

package frc.robot.states;

import frc.robot.utils.Util;

//TODO Give credit
public class SuperStructureState {

    public double height = 0; //TODO Change to min elevator height
    public double angle = 0; //TODO Change to min claw angle
    public boolean clawOpen = false;

    public boolean hasGamePiece = false; //TODO Change to hatch and ball?
    public boolean elevatorSentLastTrajectory = false;
    public boolean wristSentLastTrajectory = false;

    public SuperStructureState(double height, double angle, boolean clawOpen){
        this.height = height;
        this.angle = angle;
        this.clawOpen = clawOpen;
    }

    public SuperStructureState(double height, double angle){
        this(height, angle, false);
    }

    public SuperStructureState(SuperStructureState other){
        this.height = other.height;
        this.angle = other.angle;
        this.clawOpen = other.clawOpen;
    }

    public SuperStructureState(){
        this(0, 0, false); //TODO Change to constants
    }

    public boolean inIllegalZone(boolean allowSmallErrors){
        double allowableWristAngleError = allowSmallErrors ? 5.5 : 0; //TODO Change error allowable error
        double allowableElevatorError = allowSmallErrors ? 1 : 0; //TODO Change allowable

        if(height >= 0 + allowableElevatorError && angle < 0 - allowableWristAngleError){ //TODO Change constants
            return true;
        }

        return false;
    }

    public boolean inIllegalZone(){
        return inIllegalZone(false);
    }

    public boolean inIllegalJawZone(){
        return angle < 0 && !clawOpen;
    }

    public boolean isInRange(final SuperStructureState otherState, final double heightThreshold, double wristThreshold){
        return Util.epsilonEquals(otherState.height, height, heightThreshold) && Util.epsilonEquals(otherState.angle, angle, wristThreshold);
    }

    @Override
    public String toString(){
        return "" + height + " / " + angle + " / " + clawOpen + " / " + hasGamePiece;
    }
}
