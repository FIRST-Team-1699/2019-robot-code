package frc.robot.vision;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.Threads;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Constants;
import frc.robot.utils.MathUtils;
import frc.robot.utils.SynchronousPIDF;

public class VisionHandler {
    private static SynchronousPIDF rotatePID;

    public static void runLineUp(final NetworkTableEntry xEntry, final DifferentialDrive drivetrain, boolean test){
        rotatePID = new SynchronousPIDF(0.025, 0.002, 0.08);

        Thread threadTurn = new Thread(() -> {
            int setpointToleranceCount = 0;
            double rotate_speed;
            double angle;
            double setPoint =360;
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
        rotatePID = new SynchronousPIDF(0.3, 0.1, 0.4);
        Thread thread = new Thread(() -> {
            boolean linedUp = false;
            int iterations = 0;
            Thread.currentThread().setPriority(Thread.MIN_PRIORITY);
            VisionLight.getInstance().toggleLightState();

            //Sleep to let vision init
            try {
                Thread.sleep(500);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }

            while(!linedUp && iterations <= 0 && !Thread.interrupted()){ //TODO Changer iteration max
                //TODO Check is camera exposure can be changed on the fly and implement
                //TODO Improve efficiency
                //TODO Need to convert pixels to inches
                //TODO Add time out
                double startTime = System.nanoTime();

                System.out.println("Vision iterations: " + iterations);

                double xError ;
                try{
                    xError = ((xEntry.getDoubleArray(Constants.defaultDoubleArray)[0] + xEntry.getDoubleArray(Constants.defaultDoubleArray)[1])/2) - Constants.goalX;
                    double neededGyroChange = MathUtils.calculateNeededGyroChange(MathUtils.pixelsToInches(xError) * 12, Constants.ultrasonic.getDistance());
                    if(Double.isNaN(neededGyroChange)){
                        VisionLight.getInstance().toggleLightState();
                        return;
                    }
                    Constants.gyro.zero();
                    rotatePID.setSetpoint(neededGyroChange);

                    System.out.println("Gyro: " + Constants.gyro.getAngle() + " Gyro Setpoint: " + neededGyroChange);
                    int pidIterations = 0;
                    while(!MathUtils.checkTolerance(Constants.gyro.getAngle() - rotatePID.getSetpoint(), .5) && pidIterations < 3 && !Thread.interrupted()){
                        startTime = System.nanoTime();
                        
                        Constants.driveTrain.arcadeDrive(-rotatePID.calculate(Constants.gyro.getAngle(), .1), 0);
                        //pidIterations++; //TODO Uncomment/add timeout
                        Thread.sleep(100);

                        if(Constants.driveJoystick.getRawButton(2)){
                            VisionLight.getInstance().toggleLightState();
                            return;
                        }
                    }
                    iterations++;

                    if(Constants.driveJoystick.getRawButton(2)){
                        VisionLight.getInstance().toggleLightState();
                        return;
                    }

                    if(MathUtils.checkTolerance((((xEntry.getDoubleArray(Constants.defaultDoubleArray)[0] + xEntry.getDoubleArray(Constants.defaultDoubleArray)[1])/2) - Constants.goalX), 5)){
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
