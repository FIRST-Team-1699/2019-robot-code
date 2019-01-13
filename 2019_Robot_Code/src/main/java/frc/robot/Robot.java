package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.vision.VisionHandler;
import frc.robot.vision.VisionLight;
import frc.robot.utils.NetworkTableClient;

import static frc.robot.Constants.driveTrain;

public class Robot extends TimedRobot {

    private NetworkTableClient nTableClient;
    private NetworkTableInstance nTable;
    private NetworkTableEntry xEntry;
    private double centerX[];

    //Light var
    private boolean released = true;

    @Override
    public void robotInit() {
        //Motor Controller Definition
        Constants.portMaster = new VictorSP(Constants.portMasterPort);
        Constants.portSlave = new VictorSP(Constants.portSlavePort);
        Constants.portMaster.setInverted(true);
        Constants.portSlave.setInverted(true);
        Constants.starboardMaster = new VictorSP(Constants.starboardMasterPort);
        Constants.starboardSlave = new VictorSP(Constants.starboardSlavePort);
        SpeedControllerGroup portMotorGroup = new SpeedControllerGroup(Constants.portMaster, Constants.portSlave);
        SpeedControllerGroup starboardMotorGroup = new SpeedControllerGroup(Constants.starboardMaster, Constants.starboardSlave);
        driveTrain = new DifferentialDrive(portMotorGroup, starboardMotorGroup);

        //Elevator Test Inits
        Constants.elevator1 = new VictorSP(Constants.elevator1Port);
        Constants.elevator2 = new VictorSP(Constants.elevator2Port);
        Constants.elevator2.setInverted(true);

        //Network table variables
        nTableClient = NetworkTableClient.getInstance(); //TODO Implement or remove
        NetworkTableInstance nTable = NetworkTableInstance.getDefault();
        NetworkTable contourTable = nTable.getTable("GRIP/myContoursReport");
        xEntry = contourTable.getEntry("centerX");
        centerX = new double[0];

        //Joystick Controller Definition
        Constants.driveJoystick = new Joystick(Constants.joystickPort);

        //Start Camera
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);
        camera.setBrightness(10);
        camera.setExposureManual(10);

        //Init Light
        Constants.lightRelay = new Relay(Constants.lightRelatPort);
    }

    @Override
    public void robotPeriodic() {
        //Run Drive Base
        driveTrain.arcadeDrive(Constants.driveJoystick.getX() * -1, Constants.driveJoystick.getY()); //TODO Check correct axis

        try{
           //System.out.println(" X: " + xEntry.getDoubleArray(Constants.defaultDoubleArray)[0] + " " + xEntry.getDoubleArray(Constants.defaultDoubleArray)[1]);
        }catch (ArrayIndexOutOfBoundsException e){
           // System.out.println("No targets found.");
        }

        //Run Vision Light
        if(Constants.driveJoystick.getRawButton(2) && released){
            //VisionLight.getInstance().toggleLightState();
            System.out.println("Light State Toggled");
            released = false;
            VisionHandler.runLineUp(xEntry, driveTrain);
        }
        if(!Constants.driveJoystick.getRawButton(2)){
            released = true;
        }
    }

    @Override
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {

    }

    @Override
    public void testPeriodic() {
        //Elevator Test
        System.out.println(Constants.driveJoystick.getThrottle() + " " + Constants.driveJoystick.getTrigger());
        if(Constants.driveJoystick.getTrigger()){
            Constants.elevator1.set(Constants.driveJoystick.getThrottle());
            Constants.elevator2.set(Constants.driveJoystick.getThrottle());
        }else{
            Constants.elevator1.set(0);
            Constants.elevator2.set(0);
        }
    }
}
