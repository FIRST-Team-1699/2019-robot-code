package frc.robot.utils;

/**
 * A drive train command consisting of the left, right motor settings and whether the brake mode is enabled.
 * 
 */
public class DriveSignal {
    protected double portMotor;
    protected double starMotor;

    public DriveSignal(double left, double right) {
        this(left, right, false);

    }


    public DriveSignal(double left, double right, boolean b) {
        portMotor = left;
        starMotor = right;
    }

    public static DriveSignal NEUTRAL = new DriveSignal(0, 0);

    public double getLeft() {
        return portMotor;
    }

    public double getRight() {
        return starMotor;
    }

    @Override
    public String toString() {
        return "L: " + portMotor + ", R: " + starMotor; 
    }


}