package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.*;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.loops.Looper;
import frc.robot.subsystems.CarriageCanifier;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.DriveHelper;
import frc.robot.utils.sensors.Gyro;
import frc.robot.utils.sensors.Ultrasonic;

import java.util.Arrays;

public class Robot extends TimedRobot {
    private Looper enabledLooper = new Looper();
    private Looper disabledLooper = new Looper();

    private static final DriveHelper driveHelper = new DriveHelper();

    private boolean clawButtonReleased = false;

    //private CarriageCanifier carriageCanifier = CarriageCanifier.getInstance();
    private DriveBase driveBase = DriveBase.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Intake intake = Intake.getInstance();
    private Wrist wrist = Wrist.getInstance();

    private final SubsystemManager subsystemManager = new SubsystemManager(
          Arrays.asList(
                  DriveBase.getInstance(),
                  //CarriageCanifier.getInstance(),
                  Elevator.getInstance(),
                  Intake.getInstance(),
                  Wrist.getInstance()
          )
    );
    //private mCompressor compressor;
    private Compressor compressor;

    @Override
    public void robotInit() {
        //TODO Try catch
        //Start Camera
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);
        camera.setBrightness(50);
        camera.setExposureManual(60);
        //TODO Add exposure

        //Joystick Controller Definition
        Constants.driveJoystick = new Joystick(Constants.driveJoystickPort);
        Constants.appendageJoystick = new Joystick(Constants.appendageJoystickPort);

        //Init Light
        Constants.lightRelay = new Relay(Constants.lightRelatPort);

        //Accelerometer
        Constants.accel = new BuiltInAccelerometer();

        //Init Sensors
        Constants.gyro = new Gyro();
        Constants.gyro.calibrate();
        Constants.gyro.zero();
        Constants.ultrasonic = new Ultrasonic();

        //compressor = new Compressor(25);
        //compressor.setClosedLoopControl(true);
        //compressor.start();


        subsystemManager.registerDisabledLoops(disabledLooper);
        subsystemManager.registerEnabledLoops(enabledLooper);
    }

    @Override
    public void disabledInit(){
        enabledLooper.stop();
        DriveBase.getInstance().zeroSensors();
        elevator.zeroSensors();
        disabledLooper.start();
    }

    @Override
    public void disabledPeriodic(){
        outputToSmartDashboard();
        //TODO Reset
    }

    @Override
    public void teleopInit() {
        disabledLooper.stop();
        enabledLooper.start();
        //TODO Add more inits
    }

    @Override
    public void robotPeriodic() {
        //Update Dashboard
        updateDashboard();
    }

    @Override
    public void autonomousInit(){

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {
        //Run Drive Base

        if(Constants.appendageJoystick.getTrigger()){
            elevator.setOpenLoop(Constants.appendageJoystick.getThrottle());
        }

        //System.out.println(Constants.driveJoystick.getX() + " " + Constants.driveJoystick.getY());
        driveBase.setOpenLoop(driveHelper.genDrive(Constants.driveJoystick.getY(), Constants.driveJoystick.getX() * -1, Constants.driveJoystick.getTrigger(), true));

        wrist.setOpenLoop(Constants.appendageJoystick.getY());

        if(Constants.driveJoystick.getRawButton(2) && clawButtonReleased){
            intake.toggleClawOpen();
            clawButtonReleased = false;
        }

        if(!Constants.driveJoystick.getRawButton(2)){
            clawButtonReleased = true;
        }

        if(Constants.driveJoystick.getRawButton(7)){
            intake.shootBall(0.75); //Intake
        }else if(Constants.driveJoystick.getRawButton(8)){
            intake.shootBall(-0.75); //Shoot
        }else{
            intake.setPower(0); //Off
        }

        if(Constants.appendageJoystick.getTrigger()){
            elevator.setOpenLoop(Constants.appendageJoystick.getThrottle());
        }else{
            elevator.setOpenLoop(0);
        }
    }

    @Override
    public void testPeriodic() {
        //updateDashboard();
    }

    private void updateDashboard(){

    }

    private void outputToSmartDashboard(){
        //TODO Populate
        driveBase.outputTelemetry();
        //carriageCanifier.outputTelemetry();
        driveBase.outputTelemetry();
        elevator.outputTelemetry();
        intake.outputTelemetry();
        wrist.outputTelemetry();
    }
}
