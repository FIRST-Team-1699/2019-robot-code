package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class Ultrasonic {
    private final AnalogInput ultrasonicSensor;

    public Ultrasonic() {
        ultrasonicSensor = new AnalogInput (Constants.ultrasonicPortConstant);
    }

    public double getDistance(){
        return ultrasonicSensor.getValue() / (2 * 2.54);
    }
}
