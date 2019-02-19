package frc.robot.utils;

/**
 * A drivetrain command consiting of the left, right motor settings and wether the brake mode is enabled.
 * 
 */
public class DriveSignal {
    protected double portMotor;
    protected double starMotor;

    public DriveSignal(double left, double right) {
        this(left, right, false);

    }


    public DriveSignal(double left, double right, boolean b) {
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