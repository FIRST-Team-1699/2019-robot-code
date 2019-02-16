package frc.robot.constants;

import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.VictorSP;

public class ElevatorConstants {

    //Elevator Constants
    public static SpeedController elevator1;
    public static SpeedController elevator2;
    public static final int elevator1Port = 2;
    public static final int elevator2Port = 3;
    public static final int elevatorMasterId = 0; //TODO Change constants
    public static final int elevatorSlaveId = 0;
    public static final double epsilon = 0.0;

    //PID Constants TODO Change constants
    public static final double elevatorKP = 0.0;
    public static final double elevatorKI = 0.0;
    public static final double elevatorKD = 0.0;
    public static final double elevatorKF = 0.0;
    public static final double elevatorJogKP = 0.0;
    public static final double elevatorJogKI = 0.0;
    public static final double elevatorJogKD = 0.0;
    public static final double maxIntegralAccumulator = 0.0;
    public static final int iZone = 0;
    public static final int deadband = 0;
    public static final double feedForwardWithGamePiece = 0.0;
    public static final double feedForwardWithoutGamePiece = 0.0;

    //Elevator Motion Constants
    public static int elevatorAcceleration = 0;
    public static int elevatorVelocity = 0;

    //Ramp Rate Constants
    public static double elevatorRampRate = 0.0;
}
