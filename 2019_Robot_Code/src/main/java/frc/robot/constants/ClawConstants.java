package frc.robot.constants;

public class ClawConstants {
    public static final int master = 2;
    public static final int wristMoter = 16;

    public static final double wristKp = 3.0;
    public static final double wristKi = 0.0;
    public static final double wristKd = 50.0;
    public static final double wristKf = 1.05;
    public static final double wristJogKp = 2.0;
    public static final double wristJogKd = 40.0;
    public static final double wristKaWithCube = 0.006;
    public static final double wristKaWithoutCube = 0.003;
    public static final double wristKfMultiplierWithCube = 0.15;
    public static final double wristKfMultiplierWithoutCube = 0.1;
    public static final double wristElevatorAccelerationMultiplier = -1.0;
    public static final double wristEpsilon = 2.0;

    public static final int wristMaxIntegralAccumulator = 500000;
    public static final int wristIZone = 500;
    public static final int wristDeadband = 5;
    public static final int wristCruiseVelocity = 2500;
    public static final int wristAcceleration = 2500;
    public static final double wristRampRate = 0.001;
    public static final double autoWristRampRate = 0.01;
}
