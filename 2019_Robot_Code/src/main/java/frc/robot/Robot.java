package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.BuiltInAccelerometer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.constants.Constants;
import frc.robot.constants.ElevatorConstants;
import frc.robot.constants.PnumaticsConstants;
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
    private DoubleSolenoid ratchetSolenoid;
    boolean footLatch = true; //Foot button not pressed
    boolean footDown = false;

    boolean clawRatchetEngaged = false;
    boolean clawLatch = true; //Claw button not pressed

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
        camera.setBrightness(50);
        camera.setExposureManual(60);
        camera.setFPS(30);

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

        compressor = new Compressor(0);
        compressor.setClosedLoopControl(true);
        compressor.start();


        subsystemManager.registerDisabledLoops(disabledLooper);
        subsystemManager.registerEnabledLoops(enabledLooper);

        //TODO Move
        footMotor = new VictorSP(1);
        footSolenoid = new DoubleSolenoid(PnumaticsConstants.BigPistonOpen, PnumaticsConstants.BigPistonClosed);
        footSolenoid.clearAllPCMStickyFaults();
        footSolenoid.set(DoubleSolenoid.Value.kForward);

        ratchetSolenoid = new DoubleSolenoid(PnumaticsConstants.ClawRatchetClosed, PnumaticsConstants.ClawRatchetOpen);
        ratchetSolenoid.set(DoubleSolenoid.Value.kForward);
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
        //boolean wantWristOut = Constants.appendageJoystick.getRawButton(Constants.wantWristOutButton);
        //boolean wantWristStore = Constants.appendageJoystick.getRawButton(Constants.wantWristStoreButton);

        // if(wantWristOut){
        //     wrist.setMotionProfileAngle(-90.0); //TODO Change?
        // }else if(wantWristStore){
        //     wrist.setMotionProfileAngle(0.0); //TODO Change?
        // }else 
        if(Constants.appendageJoystick.getRawButton(Constants.wantWristManual)){
            //Manual Mode
            wrist.setOpenLoop(0.25 * Constants.appendageJoystick.getY());
        }else{
            wrist.setOpenLoop(0.0); //TODO Change?
        }

        if(Constants.driveJoystick.getRawButton(Constants.toggleHatchCups) && clawButtonReleased){
            intake.toggleClawOpen();
            clawButtonReleased = false;
        }

        if(!Constants.driveJoystick.getRawButton(Constants.toggleHatchCups)){
            clawButtonReleased = true;
        }

        if(Constants.driveJoystick.getRawButton(Constants.intakeBallButton)){
            intake.shootBall(0.5); //Intake
        }else if(Constants.driveJoystick.getRawButton(Constants.shootBallButton)){
            intake.shootBall(-0.65); //Shoot
        }else{
            //TODO Test
            intake.shootBall(0.15); //Off
        }

        //Run Elevator
        boolean wantBallScore = Constants.appendageJoystick.getRawButton(Constants.ballActionButton);
        boolean wantLowBall = Constants.appendageJoystick.getRawButton(Constants.ballActionButton) && Constants.appendageJoystick.getRawButton(Constants.wantElevatorLowButton);
        boolean wantMidBall = Constants.appendageJoystick.getRawButton(Constants.ballActionButton) && Constants.appendageJoystick.getRawButton(Constants.wantElevatorMiddleButton);
        boolean wantHighBall = Constants.appendageJoystick.getRawButton(Constants.ballActionButton) && Constants.appendageJoystick.getRawButton(Constants.wantElevatorHighButton);
        boolean wantLowHatch = Constants.appendageJoystick.getRawButton(Constants.wantElevatorLowButton);
        boolean wantMidHatch = Constants.appendageJoystick.getRawButton(Constants.wantElevatorMiddleButton);
        boolean wantHighHatch = Constants.appendageJoystick.getRawButton(Constants.wantElevatorHighButton);
        boolean wantStorage = Constants.appendageJoystick.getRawButton(Constants.wantElevatorStorageButton);

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
            elevator.setOpenLoop(Constants.appendageJoystick.getThrottle());
        }else{
            elevator.setOpenLoop(0.0); //TODO Change?
        }

        //TODO Make subsystem
        //Run Climber Foot
        if(Constants.driveJoystick.getRawButton(Constants.runClimberFootButton)){
            footMotor.set(0.7);
        } else if (Constants.driveJoystick.getRawButton(Constants.runClimberFootBackButton)) {
            footMotor.set(-0.7);
        } else {
            footMotor.set(0.0);
        }

        if(Constants.appendageJoystick.getRawButton(Constants.toggleFootPistonButton) && footLatch){
            toggleFoot();
            footLatch = false;
        }

        if(!Constants.appendageJoystick.getRawButton(Constants.toggleFootPistonButton)){
            footLatch = true;
        }

        if(Constants.appendageJoystick.getRawButton(Constants.toggleClawRatchet) && clawLatch){
            toggleArmRatchet();
            clawLatch = false;
        }

        if(!Constants.appendageJoystick.getRawButton(Constants.toggleClawRatchet)){
            clawLatch = true;
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

    //TODO Doesn't untoggle
    private void toggleArmRatchet(){
        if(!clawRatchetEngaged){
            ratchetSolenoid.set(DoubleSolenoid.Value.kReverse); //TODO Check Sides
            clawRatchetEngaged = true;
        }else{
            ratchetSolenoid.set(DoubleSolenoid.Value.kForward);
            clawRatchetEngaged = false;
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
