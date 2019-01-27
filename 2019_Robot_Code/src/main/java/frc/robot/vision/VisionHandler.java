package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.SynchronousPIDF;

import static frc.robot.Constants.driveTrain;

public class VisionHandler {
    private final PIDController pidController;
    private static SynchronousPIDF rotatePID;

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


    public static void runLineUp(final NetworkTableEntry xEntry, final DifferentialDrive drivetrain, boolean test){
        rotatePID = new SynchronousPIDF(0.025, 0.002, 0.08);

        Thread threadTurn = new Thread(() -> {
            int setpointToleranceCount = 0;
            double rotate_speed;
            double angle;
            double setPoint =-90;
            Constants.gyro.zero();
            rotatePID.setSetpoint(setPoint);
            while(//Constants.gyro.getAngle() > setPoint && 
                    DriverStation.getInstance().isEnabled()
                            &&
                            setpointToleranceCount <= 500) {
                angle = Constants.gyro.getAngle();

                rotate_speed = rotatePID.calculate(angle, .1);
                drivetrain.arcadeDrive( -rotate_speed/1,0);
                if(MathUtils.checkTolerance(setPoint - Constants.gyro.getAngle() , .5)) setpointToleranceCount++;

                //System.out.println("count: " + setpointToleranceCount);
                System.out.println("gyro angle: " + Constants.gyro.getAngle());
            }
            return;
        });
        Thread threadLight = new Thread(() -> {
            while(DriverStation.getInstance().isDisabled()) {
                VisionLight.getInstance().toggleLightState();
                try {
                    Thread.sleep(750);
                }catch(InterruptedException e){
                    e.printStackTrace();
                }


            }

        });
        threadLight.start();
        threadTurn.start();

    }

    public static void runLineUp(final NetworkTableEntry xEntry, final DifferentialDrive driveTrain){
        rotatePID = new SynchronousPIDF(0.15, 0.0, 0.2);
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

                double xError ;
                try{
                    xError = ((xEntry.getDoubleArray(Constants.defaultDoubleArray)[0] + xEntry.getDoubleArray(Constants.defaultDoubleArray)[1])/2) - Constants.goalX;
                    double neededGyroChange = MathUtils.calculateNeededGyroChange(MathUtils.pixelsToInches(xError), Constants.ultrasonic.getDistance());
                    Constants.gyro.zero();
                    //rotatePID.setSetpoint(neededGyroChange);
                    rotatePID.setSetpoint(90);

                    System.out.println("Tolerance Check: " + MathUtils.checkTolerance( Constants.gyro.getAngle() - rotatePID.getSetpoint(), .5));
                    System.out.println("Gyro: " + Constants.gyro.getAngle() + " Gyro Setpoint: " + neededGyroChange);
                    System.out.println("Distance: " + Constants.gyro.getAngle() + " change: " + MathUtils.pixelsToInches(xError));

                    int pidIterations = 0;
                    while(!MathUtils.checkTolerance(Constants.gyro.getAngle() - rotatePID.getSetpoint(), .5) && pidIterations < 3){
                        startTime = System.nanoTime();

                        System.out.println("Tolerance Check: " + MathUtils.checkTolerance(rotatePID.getSetpoint() - Constants.gyro.getAngle(), 0.5));
                        System.out.println("Gyro: " + Constants.gyro.getAngle());
                        System.out.println("xError: " + xError + " Distance: " + Constants.ultrasonic.getDistance() + " Gyro Change: " + neededGyroChange);
                        System.out.println("PID return " + rotatePID.calculate(Constants.gyro.getAngle(), System.nanoTime() - startTime) + " Time: " + ((System.nanoTime() - startTime) / 10000000));

                        Constants.driveTrain.arcadeDrive(-rotatePID.calculate(Constants.gyro.getAngle(), .1), 0);
                        //pidIterations++; //TODO Uncomment/add timeout
                        Thread.sleep(100);

                        if(Constants.driveJoystick.getRawButton(2)){
                            return;
                        }
                    }
                    iterations++;

                    if(Constants.driveJoystick.getRawButton(2)){
                        return;
                    }

                    if(MathUtils.checkTolerance((((xEntry.getDoubleArray(Constants.defaultDoubleArray)[0] + xEntry.getDoubleArray(Constants.defaultDoubleArray)[1])/2) - Constants.goalX), 10)){
                        linedUp = true;
                    }
                }catch(ArrayIndexOutOfBoundsException e){
                    System.out.println("Too few targets found");
                    return;
                }catch(InterruptedException e){
                    e.printStackTrace();
                }
            }
            VisionLight.getInstance().toggleLightState();
            return;
        });
        thread.start();
    }

}
