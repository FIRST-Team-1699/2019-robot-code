package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.SynchronousPIDF;

public class VisionHandler {
    private final PIDController pidController;
    private static SynchronousPIDF rotatePID;

    public VisionHandler(){
        this.pidController = new PIDController(0, 0, 0, new PIDCamera("none"), null); //TODO Fix PIDOutput and K values
        rotatePID = new SynchronousPIDF(0, 0, 0);
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
        rotatePID = new SynchronousPIDF();
        Thread thread = new Thread(() -> {
            boolean linedUp = false;
            int iterations = 0;

            VisionLight.getInstance().toggleLightState();

            //Sleep to let vision init
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            System.out.println("Line Up: " + linedUp + " " + " iterations: " + iterations);
            while(!linedUp && iterations <= 4){ //TODO Changer iteration max
                //TODO Check is camera exposure can be changed on the fly and implement
                //TODO Improve efficiency
                //TODO Need to convert pixels to inches
                //TODO Add time out
                double startTime = System.nanoTime();
                System.out.println("Vision iterations: " + iterations);
                double xError = 0;
                try{
                    xError = ((xEntry.getDoubleArray(Constants.defaultDoubleArray)[0] + xEntry.getDoubleArray(Constants.defaultDoubleArray)[1])/2) - Constants.goalX;
                    double neededGyroChange = MathUtils.calculateNeededGyroChange(MathUtils.pixelsToInches(xError), Constants.ultrasonic.getDistance());
                    Constants.gyro.zero();
                    rotatePID.setSetpoint(neededGyroChange);
                    System.out.println("Tolerance Check: " + MathUtils.checkTolerance(rotatePID.getSetpoint() - Constants.gyro.getAngle(), 5));
                    System.out.println("Gyro: " + Constants.gyro.getAngle());
                    while(!MathUtils.checkTolerance(rotatePID.getSetpoint() - Constants.gyro.getAngle(), 5)){
                        System.out.println("Tolerance Check: " + MathUtils.checkTolerance(rotatePID.getSetpoint() - Constants.gyro.getAngle(), 5));
                        System.out.println("Gyro: " + Constants.gyro.getAngle());
                        System.out.println("xError: " + xError + " Distance: " + Constants.ultrasonic.getDistance() + " Gyro Change: " + neededGyroChange);
                        System.out.println("PID return " + rotatePID.calculate(Constants.gyro.getAngle(), System.nanoTime() - startTime));
                        Constants.driveTrain.arcadeDrive(0, rotatePID.calculate(Constants.gyro.getAngle(), System.nanoTime() - startTime));
                    }
                    iterations++;

                    if(MathUtils.checkTolerance((((xEntry.getDoubleArray(Constants.defaultDoubleArray)[0] + xEntry.getDoubleArray(Constants.defaultDoubleArray)[1])/2) - Constants.goalX), 10)){
                        linedUp = true;
                    }
                }catch(ArrayIndexOutOfBoundsException e){
                    System.out.println("Too few targets found");
                    return;
                }
            }
            VisionLight.getInstance().toggleLightState();
            return;
        });
        thread.start();
    }

}
