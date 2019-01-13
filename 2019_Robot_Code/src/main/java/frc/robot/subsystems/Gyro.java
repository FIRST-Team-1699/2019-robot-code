package frc.robot.subsystems;

import edu.wpi.first.wpilibj.ADXRS450_Gyro;

public class Gyro {
    private static Gyro ourInstance = new Gyro();

    public static Gyro getInstance() {
        return ourInstance;
    }

    private final ADXRS450_Gyro gyro;

    private Gyro() {
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
