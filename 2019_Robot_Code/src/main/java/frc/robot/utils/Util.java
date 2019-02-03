package frc.robot.utils;

import java.util.List;

public class Util {

    public static final double epsilon = 0.0; //TODO Change value and type?

    public static double limit(double v, double maxMagnitude){
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max){
        return Math.min(max, Math.max(min, v));
    }

    public static boolean epsilonEquals(double a, double b, double epsilon){
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b){
        return epsilonEquals(a, b, 0); //TODO Change epsilon constant
    }

    public static boolean epsilonEquals(int a, int b, int epsilon){
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(final List<Double> list, double value, double epsilon){
        boolean result = true;
        for(Double valueIn : list){
            result &= epsilonEquals(valueIn, value, epsilon);
        }
        return result;
    }
}
