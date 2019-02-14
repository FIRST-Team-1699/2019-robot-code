package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.constants.Constants;
import frc.robot.constants.DriveBaseConstants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.loops.Looper;
import frc.robot.subsystems.CarriageCanifier;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.SubsystemManager;
import frc.robot.subsystems.Wrist;
import frc.robot.utils.sensors.Gyro;
import frc.robot.utils.sensors.Ultrasonic;
import frc.robot.vision.VisionHandler;
import frc.robot.utils.NetworkTableClient;

import java.util.Arrays;

public class Robot extends TimedRobot {

    //TODO Start remove./move
    private NetworkTableClient nTableClient;
    private NetworkTableInstance nTable;
    private NetworkTableEntry xEntry;
    private double centerX[];

    //Light var
    private boolean released = true;
    //TODO End remove/move

    private Looper enabledLooper = new Looper();
    private Looper disabledLooper = new Looper();

    private CarriageCanifier carriageCanifier = CarriageCanifier.getInstance();
    private DriveBase driveBase = DriveBase.getInstance();
    private Elevator elevator = Elevator.getInstance();
    private Intake intake = Intake.getInstance();
    private Wrist wrist = Wrist.getInstance();

    private final SubsystemManager subsystemManager = new SubsystemManager(
            Arrays.asList(
                    DriveBase.getInstance(),
                    CarriageCanifier.getInstance(),
                    Elevator.getInstance(),
                    Intake.getInstance(),
                    Wrist.getInstance()
            )
    );

    @Override
    public void robotInit() {
        //TODO Try catch
        //Start Camera
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);
        camera.setBrightness(10);
        camera.setExposureManual(10);

        //Start Looper
        subsystemManager.registerEnabledLoops(enabledLooper);
        subsystemManager.registerDisabledLoops(disabledLooper);



        //Motor Controller Definition
        DriveBaseConstants.portMaster = new VictorSP(DriveBaseConstants.portMasterPort);
        DriveBaseConstants.portSlave = new VictorSP(DriveBaseConstants.portSlavePort);
        DriveBaseConstants.portMaster.setInverted(true);
        DriveBaseConstants.portSlave.setInverted(true);
        DriveBaseConstants.starboardMaster = new VictorSP(DriveBaseConstants.starboardMasterPort);
        DriveBaseConstants.starboardSlave = new VictorSP(DriveBaseConstants.starboardSlavePort);
        SpeedControllerGroup portMotorGroup = new SpeedControllerGroup(DriveBaseConstants.portMaster, DriveBaseConstants.portSlave);
        SpeedControllerGroup starboardMotorGroup = new SpeedControllerGroup(DriveBaseConstants.starboardMaster, DriveBaseConstants.starboardSlave);
        DriveBaseConstants.driveTrain = new DifferentialDrive(portMotorGroup, starboardMotorGroup);

        //Elevator Test Inits
        ElevatorConstants.elevator1 = new VictorSP(ElevatorConstants.elevator1Port);
        ElevatorConstants.elevator2 = new VictorSP(ElevatorConstants.elevator2Port);
        ElevatorConstants.elevator2.setInverted(true); //TODO Change

        //Network table variables
        nTableClient = NetworkTableClient.getInstance(); //TODO Implement or remove
        NetworkTableInstance nTable = NetworkTableInstance.getDefault();
        NetworkTable contourTable = nTable.getTable("GRIP/myContoursReport");
        xEntry = contourTable.getEntry("centerX");
        centerX = new double[0];

        //Joystick Controller Definition
        Constants.driveJoystick = new Joystick(Constants.joystickPort);

        //Init Light
        Constants.lightRelay = new Relay(Constants.lightRelatPort);

        //Accelerometer
        Constants.accel = new BuiltInAccelerometer();

        //Init Sensors
        Constants.gyro = new Gyro();
        Constants.gyro.calibrate();
        Constants.gyro.zero();
        Constants.ultrasonic = new Ultrasonic();

    }

    @Override
    public void disabledInit(){
        enabledLooper.stop();
        DriveBase.getInstance().zeroSensors();
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
    public void autonomousInit() {

    }

    @Override
    public void autonomousPeriodic() {

    }

    @Override
    public void teleopPeriodic() {
        //Run Drive Base
        DriveBaseConstants.driveTrain.arcadeDrive(Constants.driveJoystick.getX() * -1, Constants.driveJoystick.getY()); //TODO Check correct axis
        //System.out.println(Constants.driveJoystick.getX() * -1);
        //System.out.println(Constants.ultrasonic.getDistance());

        //Run Vision Line Up
        if(Constants.driveJoystick.getRawButton(2) && released){
            System.out.println("Running Vision Line Up");
            released = false;
            VisionHandler.runLineUp(xEntry, DriveBaseConstants.driveTrain);
            //VisionLight.getInstance().toggleLightState();
        }
        if(!Constants.driveJoystick.getRawButton(2)){
            released = true;
        }

    }

    @Override
    public void testPeriodic() {
        //Elevator Test
        System.out.println(Constants.driveJoystick.getThrottle() + " " + Constants.driveJoystick.getTrigger());
        if(true){//Constants.driveJoystick.getTrigger()){
            ElevatorConstants.elevator1.set(-Constants.driveJoystick.getThrottle());
            ElevatorConstants.elevator2.set(Constants.driveJoystick.getThrottle());
        }else{
            ElevatorConstants.elevator1.set(0);
            ElevatorConstants.elevator2.set(0);
        }
        //updateDashboard();
    }


    int testCounter = 0;
    private void updateDashboard(){
        if (testCounter++ > 200){
            System.out.println("X: " + Constants.accel.getX() + " Y: " + Constants.accel.getY() + " Z: " + Constants.accel.getZ());
            testCounter =0;
        }
        
    }

    private void outputToSmartDashboard(){
        //TODO Populate
        carriageCanifier.outputTelemetry();
        driveBase.outputTelemetry();
        elevator.outputTelemetry();
        intake.outputTelemetry();
        wrist.outputTelemetry();
    }
}
