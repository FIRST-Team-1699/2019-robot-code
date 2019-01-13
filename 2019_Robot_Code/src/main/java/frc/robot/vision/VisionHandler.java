package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;

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

    public static void runLineUp(final NetworkTableEntry xEntry, final DifferentialDrive driveTrain){
        Thread thread = new Thread(() -> {
            boolean linedUp = false;

            VisionLight.getInstance().toggleLightState();

            //Sleep to let vision init
            try {
                Thread.sleep(2000);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            while(!linedUp){
                System.out.println(linedUp);
                if((xEntry.getDoubleArray(Constants.defaultDoubleArray)[0] + xEntry.getDoubleArray(Constants.defaultDoubleArray)[0])/2 > (640/2)){
                    driveTrain.arcadeDrive(.35, 0);
                }else{
                    driveTrain.arcadeDrive(-.35, 0);
                }
            }
            VisionLight.getInstance().toggleLightState();
            return;
        });
        thread.start();
    }

}
