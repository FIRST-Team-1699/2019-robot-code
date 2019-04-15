//Credit for code to FRC Team 254
//https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/frc2018/states/IntakeState.java

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

    public boolean seesBall(){
        return false; //TODO Implement
    }

    public void setPower(double power) {
        leftMotor = rightMotor = power;
    }
}
