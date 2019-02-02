package frc.robot.states;

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
        this(0, 0, false); //TODO Change to mins
    }

    public boolean inIllegalZone(boolean allowSmallErrors){
        return false; //TODO Implement
    }

    public boolean inIllegalZone(){
        return inIllegalZone(false);
    }

    public boolean inIllegalJawZone(){
        return false;
    }

    public boolean isInRange(final SuperStructureState state, final double heightThreshold, double wristThreshold){
        return false; //TODO Implement
    }

    @Override
    public String toString(){
        return "" + height + " / " + angle + " / " + clawOpen + " / " + hasGamePiece;
    }
}
