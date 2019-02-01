package frc.robot.planners;

import frc.robot.states.SuperStructureState;

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
        public WaitForElevatorSafeSubcommand(SuperStructureState endState) {
            super(endState);
        }
    }

    class WaitForElevatorApproachingSubcommand extends SubCommand{
        public WaitForElevatorApproachingSubcommand(SuperStructureState endState) {
            super(endState);
        }
    }

    class WaitForFinalSetpointSubcommand extends SubCommand{
        public WaitForFinalSetpointSubcommand(SuperStructureState endState) {
            super(endState);
        }
    }

    protected SuperStructureState commandedState = new SuperStructureState();
    protected SuperStructureState intermediateCommandedState = new SuperStructureState();
    protected LinkedList<SubCommand> commandQueue = new LinkedList<>();
    protected Optional<SubCommand> currentCommand = Optional.empty();
}
