package frc.robot.constants;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveBaseConstants {
    //Drive Train Constants
    public static VictorSP portMaster;
    public static VictorSP portSlave;
    public static VictorSP starboardMaster;
    public static VictorSP starboardSlave;
    public static final int portMasterPort = 4;
    public static final int portSlavePort = 5;
    public static final int starboardMasterPort = 0;
    public static final int starboardSlavePort = 1;
    public static DifferentialDrive driveTrain;
}
