package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Constants {

    //Motor Controller and Drive Train Constants
    public static VictorSP portMaster;
    public static VictorSP portSlave;
    public static VictorSP starboardMaster;
    public static VictorSP starboardSlave;

    public static final int portMasterPort = 0;
    public static final int portSlavePort = 1;
    public static final int starboardMasterPort = 2;
    public static final int starboardSlavePort = 3;

    public static DifferentialDrive driveTrain;

    //Joystick Constants
    public static final int joystickPort = 0;
    public static Joystick driveJoystick;

    //Team Constants
    public static final String teamNumber = "1699";
}
