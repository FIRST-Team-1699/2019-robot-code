//Credit for code to FRC Team 254
//https://github.com/Team254/FRC-2018-Public/blob/master/src/main/java/com/team254/frc2018/loops/Loop.java

package frc.robot.loops;

public interface Loop {

    void onStart(double timestamp);
    void onLoop(double timestamp);
    void onStop(double timestamp);
}
