package frc.robot.vision;

import edu.wpi.first.wpilibj.PIDController;

public class VisionHandler {
    private final PIDController pidController;

    public VisionHandler(){
        this.pidController = new PIDController(0, 0, 0, new PIDCamera("none"), null); //TODO Fix PIDOutput and K values
    }

    public void startPID(){
        pidController.enable();
    }

    public void stopPID(){
        pidController.disable();
    }

    public double driveOutput(){
        return pidController.get(); //TODO Test correct output
    }
}
