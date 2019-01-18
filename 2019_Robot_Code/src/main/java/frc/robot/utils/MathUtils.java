package frc.robot.utils;

public class MathUtils {

    public static double calculateNeededGyroChange(final double visionError, final double distance){
        return Math.atan(visionError/distance); //TODO Write test
    }

    public static boolean checkTolerance(final double error, final double tolerance) {
        return (Math.abs(error) - tolerance) <= 0;
    }

    //TODO Add pixel to inch algorithm: https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/288985-identifying-and-processing-the-targets
    public static double pixelsToInches(final double number){ //TODO Change var names
        return 0; //TODO Implement
    }
}
