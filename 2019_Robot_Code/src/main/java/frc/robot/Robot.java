package frc.robot;

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
        Constants.starboardMaster = new VictorSP(Constants.starboardMasterPort);
        Constants.starboardSlave = new VictorSP(Constants.starboardSlavePort);

        SpeedControllerGroup portMotorGroup = new SpeedControllerGroup(Constants.portMaster, Constants.portSlave);
        SpeedControllerGroup starboardMotorGroup = new SpeedControllerGroup(Constants.starboardMaster, Constants.starboardSlave);

        Constants.driveTrain = new DifferentialDrive(portMotorGroup, starboardMotorGroup);

        //Joystick Controller Definition
        Constants.driveJoystick = new Joystick(Constants.joystickPort);
    }

    @Override
    public void robotPeriodic() {
        
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
