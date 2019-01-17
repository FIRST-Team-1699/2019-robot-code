package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Gyro {
    private final ADXRS450_Gyro gyro;

    public Gyro() {
        this.gyro = new ADXRS450_Gyro();
    }

    public double getAngle(){
        return gyro.getAngle();
    }

    public void calibrate(){
        gyro.calibrate();
    }

    public void zero(){
        gyro.reset();
    }
}
