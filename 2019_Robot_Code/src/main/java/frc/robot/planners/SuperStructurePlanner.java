package frc.robot.planners;

import frc.robot.states.SuperStructureState;

public class SuperStructurePlanner {
    private boolean upwardsSubcoCommandEnabled = true;

    class SubCommand {
        public SubCommand(SuperStructureState endState){
            endState = endState;
        }

        public SuperStructureState endState;
    }
}
