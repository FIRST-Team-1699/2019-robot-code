package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.loops.Looper;
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

    //TODO Move to subsystem
    private VictorSP footMotor;
    private DoubleSolenoid footSolenoid;
    boolean footLatch = true; //Foot button not pressed
    boolean footDown = false;

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
   // private Compressor compressor;

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

        //TODO Move
        footMotor = new VictorSP(3);
        footSolenoid = new DoubleSolenoid(2, 3);
        footSolenoid.clearAllPCMStickyFaults();
        footSolenoid.set(DoubleSolenoid.Value.kForward); //TODO Check
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
        disabledLooper.stop();
        enabledLooper.start();
    }

    @Override
    public void autonomousPeriodic() {
        runRobot();
    }

    @Override
    public void teleopPeriodic() {
        runRobot();
    }

    private void runRobot(){
        outputToSmartDashboard();

        //Run Drive Base
        driveBase.setOpenLoop(driveHelper.genDrive(Constants.driveJoystick.getY(), Constants.driveJoystick.getX() * -1, Constants.driveJoystick.getTrigger(), true));

        //Run Wrist
        boolean wantWristOut = Constants.appendageJoystick.getRawButton(5);
        boolean wantWristStore = Constants.appendageJoystick.getRawButton(3);

        if(wantWristOut){
            wrist.setMotionProfileAngle(-90.0); //TODO Change?
        }else if(wantWristStore){
            wrist.setMotionProfileAngle(0.0); //TODO Change?
        }else if(Constants.appendageJoystick.getRawButton(6)){
            //Manual Mode
            wrist.setOpenLoop(Constants.appendageJoystick.getY());
        }else{
            wrist.setOpenLoop(0.0); //TODO Change?
        }

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

        //Run Elevator
        boolean wantBallScore = Constants.appendageJoystick.getRawButton(2);
        boolean wantLowBall = Constants.appendageJoystick.getRawButton(2) && Constants.appendageJoystick.getRawButton(7);
        boolean wantMidBall = Constants.appendageJoystick.getRawButton(2) && Constants.appendageJoystick.getRawButton(9);
        boolean wantHighBall = Constants.appendageJoystick.getRawButton(2) && Constants.appendageJoystick.getRawButton(11);
        boolean wantLowHatch = Constants.appendageJoystick.getRawButton(7);
        boolean wantMidHatch = Constants.appendageJoystick.getRawButton(9);
        boolean wantHighHatch = Constants.appendageJoystick.getRawButton(11);
        boolean wantStorage = Constants.appendageJoystick.getRawButton(8);

        if(wantLowBall){
            elevator.setMotionMagicPosition(ElevatorConstants.ballLow);
        }else if(wantMidBall){
            elevator.setMotionMagicPosition(ElevatorConstants.ballMiddle);
        }else if(wantHighBall){
            elevator.setMotionMagicPosition(ElevatorConstants.ballHigh);
        }else if(wantLowHatch){
            elevator.setMotionMagicPosition(ElevatorConstants.hatchLow);
        }else if(wantMidHatch){
            elevator.setMotionMagicPosition(ElevatorConstants.hatchMiddle);
        }else if(wantHighHatch){
            elevator.setMotionMagicPosition(ElevatorConstants.hatchHigh);
        }else if(wantStorage) {
            elevator.setMotionMagicPosition(ElevatorConstants.initialHeight); //TODO Change?
        }else if(wantBallScore){
            elevator.setMotionMagicPosition(ElevatorConstants.climbHeight);
        }else if(Constants.appendageJoystick.getTrigger()){
            //Manual Mode
            elevator.setOpenLoop(Constants.driveJoystick.getThrottle());
        }else{
            elevator.setOpenLoop(0.0); //TODO Change?
        }

        //TODO Make subsystem
        //Run Climber Foot
        if(Constants.driveJoystick.getRawButton(11)){
            footMotor.set(0.7); //TODO Check direction
        } else if (Constants.driveJoystick.getRawButton(12)) {
            footMotor.set(-0.7);
        } else {
            footMotor.set(0.0);
        }

        if(Constants.appendageJoystick.getRawButton(4) && footLatch){
            toggleFoot();
            footLatch = false;
        }

        if(!Constants.appendageJoystick.getRawButton(4)){
            footLatch = true;
        }
    }

    private void toggleFoot() {
        if(!footDown){
            footSolenoid.set(DoubleSolenoid.Value.kReverse); //TODO Check Sides
            footDown = true;
        }else{
            footSolenoid.set(DoubleSolenoid.Value.kForward);
            footDown = false;
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
