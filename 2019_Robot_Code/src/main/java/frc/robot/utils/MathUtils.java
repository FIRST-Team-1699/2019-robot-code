package frc.robot.utils;

import frc.robot.Constants;

public class MathUtils {

    public static double calculateNeededGyroChange(final double visionError, final double distance){
        System.out.println("DISTANCE CALC --------- Vision Error: " + visionError + " Distance: " + distance);
        return (Math.atan(Math.toDegrees(Math.abs(visionError)/distance))) * (visionError / Math.abs(visionError)); //TODO Write test
    }

    public static boolean checkTolerance(final double error, final double tolerance) {
        return (Math.abs(error) - tolerance) <= 0;
    }

    //https://wpilib.screenstepslive.com/s/currentCS/m/vision/l/288985-identifying-and-processing-the-targets
    //FOVft = 2*w = 2*d*tanΘ
    //Tft*FOVpixel/(2*Tpixel*tanΘ)
    public static double pixelsToInches(final double targetPixels){ //TODO Change var names
        double distance = (Constants.targetWidth * Constants.imageWidth)/(2 * targetPixels * Math.tan((Constants.cameraFieldOfView / 2)));
        return distance * Math.tan(Constants.cameraFieldOfView / 2);
    }
}
