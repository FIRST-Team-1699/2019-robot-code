package frc.robot.utils;

public class MathUtils {

    public static double calculateNeededGyroChange(final double visionError, final double distance){
        return Math.atan(visionError/distance); //TODO Write test
    }

    public static boolean checkTolerance(final double error, final double tolerance){
        return (Math.abs(error) - tolerance) <= 0;
    }
}
