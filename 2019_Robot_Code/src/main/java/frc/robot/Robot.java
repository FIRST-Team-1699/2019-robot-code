package frc.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Robot extends TimedRobot {

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
        Constants.driveTrain = new DifferentialDrive(portMotorGroup, starboardMotorGroup);

        //Joystick Controller Definition
        Constants.driveJoystick = new Joystick(Constants.joystickPort);

        //Start Camera
        UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
        camera.setResolution(640, 480);
        camera.setBrightness(10);
        camera.setExposureManual(10);
    }

    @Override
    public void robotPeriodic() {
        //Run Drive Base
        Constants.driveTrain.arcadeDrive(Constants.driveJoystick.getX() * -1, Constants.driveJoystick.getY()); //TODO Check correct axis
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

    }
}
