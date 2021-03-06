package frc.robot.constants;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Relay;
import edu.wpi.first.wpilibj.interfaces.Accelerometer;
import frc.robot.utils.sensors.Gyro;
import frc.robot.utils.sensors.Ultrasonic;

public class Constants {

    //A general list of constants

    //Accelerometer
    public static Accelerometer accel;

    //Joystick Constants
    public static final int driveJoystickPort = 0;
    public static final int appendageJoystickPort = 1;
    public static Joystick driveJoystick;
    public static Joystick appendageJoystick;

    //Team Constants
    public static final String teamNumber = "1699";

    //Ultrasonic Sensor Constant
    public static final int ultrasonicPortConstant = 0;

    //Network Table Constants
    public static final double[] defaultDoubleArray = {0.0, 0.0, 0.0, 0.0};


    //Vision Constants
    public static Relay lightRelay;
    public static final int lightRelatPort = 0;
    public static final double goalX = 320; //TODO Calculate actual goal x
    public static final double visionTolerance = .5; //TODO Update to actual tolerance and establish units
    public static double distanceBetweenVisionTargets = 5; //In inches //TODO Updated value
    public static final double cameraFieldOfView = 44.48; //TODO Double check value
    public static final double targetWidth = 4/12.0; //feet //TODO Update value
    public static final double imageWidth = 640; //TODO Double check value

    //Sensors
    public static Gyro gyro;
    public static Ultrasonic ultrasonic;

    //CAN Constants
    public static final int longCANTimeoutMs = 0; //TODO Change constant

    //Joystick constants
    public static final int wantWristOutButton = 5;
    public static final int wantWristStoreButton = 4;
    public static final int wantWristManual = 6;
    public static final int toggleHatchCups = 2;
    public static final int intakeBallButton = 7;
    public static final int shootBallButton = 8;
    public static final int ballActionButton = 2;
    public static final int wantElevatorLowButton = 7;
    public static final int wantElevatorMiddleButton = 9;
    public static final int wantElevatorHighButton = 11;
    public static final int wantElevatorStorageButton = 8;
    public static final int runClimberFootButton = 11;
    public static final int runClimberFootBackButton = 12;
    public static final int toggleFootPistonButton = 3;
    public static final int toggleClawRatchet = 10;

}
