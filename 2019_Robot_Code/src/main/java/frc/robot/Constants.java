package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.subsystems.Gyro;
import frc.robot.subsystems.Ultrasonic;

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
    public static final double goalX = 320; //TODO Calculate actual goal x
    public static final double visionTolerance = 10; //TODO Update to actual tolerance and establish units
    public static double distanceBetweenVisionTargets = 5; //In inches //TODO Updated value
    public static final double cameraFieldOfView = 68.5; //TODO Double check value
    public static final double targetWidth = 0; //TODO Update value
    public static final double imageWidth = 640; //TODO Double check value

    //Sensors
    public static Gyro gyro;
    public static Ultrasonic ultrasonic;
}
