package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;

public class PIDDriveTrain implements PIDOutput {

    private final RobotDriveBase driveTrain;

    public PIDDriveTrain(final RobotDriveBase driveTrain){
        this.driveTrain = driveTrain;
    }

    @Override
    public void pidWrite(double output) {

    }
}
