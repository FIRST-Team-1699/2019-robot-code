package frc.robot.states;

public class IntakeState {
    public enum JawState {
        Open,
        Closed
    }

    public JawState jawState = JawState.Closed;
    public double leftMotor = 0.0;
    public double rightMotor = 0.0;
    public double wristAngle = 0.0;
    public double wristSetpoint = 0;

    public boolean leftSensorTriggered = false;
    public boolean rightSensorTriggered = false;

    public void setPower(double power) {
        leftMotor = rightMotor = power;
    }
}
