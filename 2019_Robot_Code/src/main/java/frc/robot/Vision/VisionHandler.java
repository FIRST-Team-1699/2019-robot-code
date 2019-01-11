package frc.robot.Vision;

import edu.wpi.first.wpilibj.drive.RobotDriveBase;

public class VisionHandler {

    private final RobotDriveBase driveTrain;

    public VisionHandler(final RobotDriveBase driveTrain){
        this.driveTrain = driveTrain;
    }

    public boolean alignToTarget(){
        return false; //TODO Populate with code to align to vision target
    }
}
