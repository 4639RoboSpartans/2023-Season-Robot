package frc.robot.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.network.Network;

public class LimeLight {
    public void getOffsetFromCenteredAprilTag(){
        var table = Network.getTable("campose");
        // TODO: finish implementation
        SmartDashboard.putNumber("tv", table.getEntry("tv").getDouble(0));
        SmartDashboard.putNumber("tx", table.getEntry("tx").getDouble(0));
        SmartDashboard.putNumber("ty", table.getEntry("ty").getDouble(0));
        SmartDashboard.putNumber("ta", table.getEntry("ta").getDouble(0));
        
    }
}

