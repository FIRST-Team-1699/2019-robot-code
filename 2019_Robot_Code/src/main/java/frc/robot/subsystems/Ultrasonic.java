package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class Ultrasonic {
    private static Ultrasonic ourInstance = new Ultrasonic();

    public static Ultrasonic getInstance() {
        return ourInstance;
    }

    private final AnalogInput ultrasonicSensor;

    private Ultrasonic() {
        ultrasonicSensor = new AnalogInput (Constants.ultrasonicPortConstant);
    }

    public double getDistance(){
        return ultrasonicSensor.getValue() / (2 * 2.54);
    }
}
