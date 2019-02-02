package frc.robot.utils;

public class Util {

    public static double limit(double v, double maxMagnitude){
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max){
        return Math.min(max, Math.max(min, v));
    }
}
