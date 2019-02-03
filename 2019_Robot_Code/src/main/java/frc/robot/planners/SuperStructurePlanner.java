package frc.robot.planners;

import frc.robot.states.SuperStructureState;
import frc.robot.utils.Util;

import java.util.LinkedList;
import java.util.Optional;

public class SuperStructurePlanner {
    private boolean upwardsSubCommandEnabled = true;

    class SubCommand {
        public SubCommand(SuperStructureState endState){
            this.endState = endState;
        }

        public SuperStructureState endState;
        public double heightThreshold = 0; //TODO Change
        public double wristThreshold = 0; //TODO Change

        public boolean isFinished(SuperStructureState currentState){
            return endState.isInRange(currentState, heightThreshold, wristThreshold); //TODO Implement
        }
    }

    class WaitForWristSafeSubcommand extends SubCommand{

        public WaitForWristSafeSubcommand(final SuperStructureState endState) {
            super(endState);
            wristThreshold = wristThreshold + Math.max(0.0, endState.angle - 0); //TODO Add clear stage wrist angle
        }

        @Override
        public boolean isFinished(SuperStructureState currentState){
            return endState.isInRange(currentState, Double.POSITIVE_INFINITY, wristThreshold);
        }
    }

    class WaitForElevatorSafeSubcommand extends SubCommand{
        public WaitForElevatorSafeSubcommand(SuperStructureState endState, SuperStructureState currentState) {
            super(endState);
            if(endState.height >= currentState.height){
                heightThreshold = heightThreshold + Math.max(0.0, endState.height - 0); //TODO Change constants
            }else{
                heightThreshold = heightThreshold + Math.max(0.0, 0 - endState.height); //TODO Change constants
            }
        }

        @Override
        public boolean isFinished(SuperStructureState currentState){
            return endState.isInRange(currentState, heightThreshold, Double.POSITIVE_INFINITY);
        }
    }

    class WaitForElevatorApproachingSubcommand extends SubCommand{
        public WaitForElevatorApproachingSubcommand(SuperStructureState endState) {
            super(endState);
            heightThreshold = 0; //TODO Change constant
        }

        @Override
        public boolean isFinished(SuperStructureState currentState){
            return endState.isInRange(currentState, heightThreshold, Double.POSITIVE_INFINITY);
        }
    }

    class WaitForFinalSetpointSubcommand extends SubCommand{
        public WaitForFinalSetpointSubcommand(SuperStructureState endState) {
            super(endState);
        }

        @Override
        public boolean isFinished(SuperStructureState currentState){
            return currentState.elevatorSentLastTrajectory && currentState.wristSentLastTrajectory;
        }
    }

    protected SuperStructureState commandedState = new SuperStructureState();
    protected SuperStructureState intermediateCommandedState = new SuperStructureState();
    protected LinkedList<SubCommand> commandQueue = new LinkedList<>();
    protected Optional<SubCommand> currentCommand = Optional.empty();

    public synchronized boolean setDesiredState(SuperStructureState desiredStateIn, SuperStructureState currentState){
        SuperStructureState desiredState = new SuperStructureState(desiredStateIn);

        desiredState.angle = Util.limit(desiredState.angle, 0, 0); //TODO Change Limits
        desiredState.height = Util.limit(desiredState.height, 0, 0); //TODO Change Limits

        SuperStructureState swapJaw = new SuperStructureState(currentState);
        swapJaw.clawOpen = desiredState.clawOpen;

        if(desiredState.inIllegalZone() || swapJaw.inIllegalZone()){
            return false;
        }

        commandQueue.clear();

        final boolean longUpwardsMove = desiredState.height - currentState.height > 0; //TODO Change height constant
        final double firstWristAngle = longUpwardsMove ? Math.min(desiredState.angle, 0) : desiredState.angle; //TODO Change constant

        if(currentState.angle < 0 && desiredState.height > 0){ //TODO Change constant
            //TODO Add code
        }else if(desiredState.angle < 0 && currentState.height > 0){ //TODO Change constant
            //TODO Add code
        }

        if(longUpwardsMove){
            if(upwardsSubCommandEnabled){
                commandQueue.add(new WaitForElevatorApproachingSubcommand(new SuperStructureState(desiredState.height, firstWristAngle, true)));
            }
        }

        commandQueue.add(new WaitForFinalSetpointSubcommand(desiredState));

        currentCommand = Optional.empty();

        return true;
    }

    void reset(SuperStructureState currentState){
        intermediateCommandedState = commandedState;
        commandQueue.clear();
        currentCommand = Optional.empty();
    }

    public boolean isFinished(SuperStructureState currentState){
        return currentCommand.isPresent() && commandQueue.isEmpty() && currentState.wristSentLastTrajectory && currentState.elevatorSentLastTrajectory;
    }

    public synchronized void setUpwardsSubCommandEnabled(boolean enabled){
        upwardsSubCommandEnabled = enabled;
    }

    public SuperStructureState updaet(SuperStructureState currentState){
        if(!currentCommand.isPresent() && !commandQueue.isEmpty()){
            currentCommand = Optional.of(commandQueue.remove());
        }

        if(currentCommand.isPresent()){
            SubCommand subCommand = currentCommand.get();
            intermediateCommandedState = subCommand.endState;
            if(subCommand.isFinished(currentState) && !commandQueue.isEmpty()){
                currentCommand = Optional.empty();
            }
        }else{
            intermediateCommandedState = currentState;
        }

        commandedState.angle = Util.limit(intermediateCommandedState.angle, 0, 0); //TODO Change constants
        commandedState.height = Util.limit(intermediateCommandedState.height, 0, 0); //TODO Change constants

        return commandedState;
    }
}
