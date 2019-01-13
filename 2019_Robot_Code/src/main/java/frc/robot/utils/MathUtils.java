package frc.robot.utils;

public class MathUtils {

    public double calulateNeededGyroChange(double visionError, double distance){
        return Math.atan(visionError/distance);
    }
}
