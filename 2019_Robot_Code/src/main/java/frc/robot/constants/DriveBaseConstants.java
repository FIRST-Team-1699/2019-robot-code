package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveBaseConstants {
    //Drive Train Constants
    public static WPI_TalonSRX portMaster;
    public static WPI_TalonSRX portSlave;
    public static WPI_TalonSRX starboardMaster;
    public static WPI_TalonSRX starboardSlave;
    public static final int portMasterPort = 12;
    public static final int portSlavePort = 13;
    public static final int starboardMasterPort = 10;
    public static final int starboardSlavePort = 11;
    public static DifferentialDrive driveTrain;

    public static final double pConstant = 0.4;
    public static final double iConstant = 0.25;
    public static final double dConstant = 0.0;
}
