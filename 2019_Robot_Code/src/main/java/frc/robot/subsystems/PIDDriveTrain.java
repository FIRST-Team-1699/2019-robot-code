package frc.robot.subsystems;

import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class PIDDriveTrain implements PIDOutput {

    private final DifferentialDrive driveTrain;

    public PIDDriveTrain(final DifferentialDrive driveTrain){
        this.driveTrain = driveTrain;
    }

    @Override
    public void pidWrite(double output) {
        driveTrain.arcadeDrive(0, output);
    }
}
