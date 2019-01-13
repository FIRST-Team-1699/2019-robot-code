package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class Constants {

    //Drive Train Constants
    public static VictorSP portMaster;
    public static VictorSP portSlave;
    public static VictorSP starboardMaster;
    public static VictorSP starboardSlave;

    public static final int portMasterPort = 4;
    public static final int portSlavePort = 5;
    public static final int starboardMasterPort = 0;
    public static final int starboardSlavePort = 1;

    public static DifferentialDrive driveTrain;

    //Elevator Constants
    public static VictorSP elevator1;
    public static VictorSP elevator2;

    public static final int elevator1Port = 2;
    public static final int elevator2Port = 3;

    //Joystick Constants
    public static final int joystickPort = 0;
    public static Joystick driveJoystick;

    //Team Constants
    public static final String teamNumber = "1699";

    //PID Controller Goal Constants
    public static final double visionGoalConstant = 0; //TODO Change if needed

    //Ultrasonic Sensor Constant
    public static final int ultrasonicPortConstant = 0;

    //Network Table Constants
    public static final double[] defaultDoubleArray = {0.0, 0.0, 0.0, 0.0};

    //Vision Constants
    public static Relay lightRelay;
    public static final int lightRelatPort = 0;
}
