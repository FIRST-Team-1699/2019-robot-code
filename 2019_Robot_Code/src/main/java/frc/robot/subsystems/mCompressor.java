package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.constants.PnumaticsConstants;

public class mCompressor extends Subsystem{
    private static mCompressor mCompressorInstance = null;
    private Compressor compressor;

    private mCompressor(){
        compressor = new Compressor(PnumaticsConstants.pcmid);
    }
    public static mCompressor getInstance(){
        if(mCompressorInstance == null)
            mCompressorInstance = new mCompressor();
            return mCompressorInstance;
    }
	@Override
	public boolean checkSystem() {
		return compressor.getClosedLoopControl();
	}
	@Override
	public void outputTelemetry() {
		SmartDashboard.putBoolean("Compressor_Open_Loop_Control", compressor.getClosedLoopControl());
	}
	@Override
	public void stop() {
		 compressor.setClosedLoopControl(false);
    }
    
    public void start(){
        compressor.setClosedLoopControl(true);
    }
}

