package frc.robot.vision;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import frc.robot.utils.NetworkTableClient;

public class PIDCamera implements PIDSource {

    private final String ntEntryKey;
    
    public PIDCamera(final String ntEntryKey){
        this.ntEntryKey = ntEntryKey;
        NetworkTableClient.getInstance().addEntryKey(ntEntryKey);
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {}

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return NetworkTableClient.getInstance().getEntry(ntEntryKey).getDouble(0);
    }
}