package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AnalogInput;
import frc.robot.Constants;

public class Ultrasonic {
    private final AnalogInput ultrasonicSensor;
    private final double scalar = 4.84/512.0;

    public Ultrasonic() {
        ultrasonicSensor = new AnalogInput (Constants.ultrasonicPortConstant);
    }

    public double getDistance(){
        return ultrasonicSensor.getVoltage() / (scalar * 2.54);
    }
}
