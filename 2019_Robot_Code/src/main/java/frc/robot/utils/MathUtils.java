package frc.robot.utils;

import frc.robot.Constants;

public class MathUtils {

    public static double calculateNeededGyroChange(final double visionError, final double distance){
        return Math.atan(visionError/distance); //TODO Write test
    }

    public static boolean checkTolerance(final double error, final double tolerance) {
        return (Math.abs(error) - tolerance) <= 0;
    }

    //https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/288985-identifying-and-processing-the-targets
    public static double pixelsToInches(final double targetPixels){ //TODO Change var names
        double distance = (Constants.targetWidth * Constants.imageWidth)/(2 * targetPixels * Math.tan((Constants.cameraFieldOfView / 2)));
        System.out.println(distance);
        return distance; //* Math.tan(Math.toRadians(Constants.cameraFieldOfView / 2));
    }
}
