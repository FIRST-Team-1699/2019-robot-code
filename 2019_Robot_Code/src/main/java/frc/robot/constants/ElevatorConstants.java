package frc.robot.constants;

public class ElevatorConstants {

    //Elevator Constants

    //Motor ids
    public static final int elevatorMasterId = 14;
    public static final int elevatorSlaveId = 15;


    public static final double epsilon = 1.0;

    //PID Constants
    public static final double elevatorKP = 0.15;
    public static final double elevatorKI = 0.0;
    public static final double elevatorKD = 4.0;
    public static final double elevatorKF = 0.06;
    public static final double elevatorJogKP = 0.1;
    public static final double elevatorJogKI = 0.0;
    public static final double elevatorJogKD = 3.0;
    public static final double maxIntegralAccumulator = 500000.0;
    public static final int iZone = 0;
    public static final int deadband = 0;
    public static final double feedForwardWithGamePiece = -0.06;
    public static final double feedForwardWithoutGamePiece = -0.07;

    //Elevator Motion Constants
    public static final int elevatorAcceleration = 10000;
    public static final int elevatorVelocity = 5000;

    ///Elevator Heights
    public static final double initialHeight = 5.0;
    public static final double ballLow = 27.5;
    public static final double ballMiddle = 55.5;
    public static final double ballHigh = 83.5;
    public static final double hatchLow = 19.0;
    public static final double hatchMiddle = 47.0;
    public static final double hatchHigh = 75.0;
    public static final double climbHeight = 20.0;

    //Ramp Rate Constants
    public static double elevatorRampRate = 0.1;
}
