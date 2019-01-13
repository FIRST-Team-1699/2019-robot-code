package frc.robot.vision;

import edu.wpi.first.wpilibj.Relay;
import frc.robot.Constants;

public class VisionLight {
    private static VisionLight ourInstance = new VisionLight();

    public static VisionLight getInstance() {
        return ourInstance;
    }

    private VisionLight() {}

    private LightState lightState = LightState.OFF;

    public enum LightState{
        ON,
        OFF
    }

    public void toggleLightState(){
        if(lightState == LightState.ON){
            lightState = LightState.OFF;
            Constants.lightRelay.set(Relay.Value.kOff);
        }else{
            lightState = LightState.ON;
            Constants.lightRelay.set(Relay.Value.kForward);
        }
    }

    public LightState getLightState(){
        return lightState;
    }
}
