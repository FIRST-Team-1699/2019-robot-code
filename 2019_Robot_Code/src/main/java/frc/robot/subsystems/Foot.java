package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.VictorSP;
import frc.robot.constants.FootConstants;

public class Foot extends Subsystem{

    private static Foot instance;

    public static Foot getInstance(){
        if(instance == null){
            instance = new Foot();
        }
        return instance;
    }

    private final VictorSP footMotor;
    private final DoubleSolenoid footSolenoid;

    private Foot(){
        footMotor = new VictorSP(FootConstants.footMotorPort);
        footSolenoid = new DoubleSolenoid(FootConstants.footSolenoidPort1, FootConstants.footSolenoidPort2);
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }
}
