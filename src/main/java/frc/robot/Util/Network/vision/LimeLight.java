package frc.robot.util.network.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.util.network.Network;

public class LimeLight {
    
    public static void getOffsetFromCenteredAprilTag(){
        var table = Network.getTable("limelight");
        // TODO: finish implementation
        SmartDashboard.putNumber("tv", table.getDouble("tv"));
        SmartDashboard.putNumber("tx", table.getDouble("tx"));
        SmartDashboard.putNumber("ty", table.getDouble("ty"));
        SmartDashboard.putNumber("tid", table.getDouble("tid"));
        var a = table.getDoubleArray("botpose");
        if(a.length != 6) a = new double[]{-1, -1, -1, -1, -1, -1};
        
        // X = left-right, Y = up-down, Z = forward-backward
        // All rotations are in degrees
        SmartDashboard.putNumber("z distance", a[0]);
        SmartDashboard.putNumber("x distance", a[1]);
        SmartDashboard.putNumber("y distance", a[2]);
        SmartDashboard.putNumber("z rotation", a[3]);
        SmartDashboard.putNumber("x rotation", a[4]);
        SmartDashboard.putNumber("y rotation", a[5]);

        

        var b = table.getDoubleArray("campose"); 
        if(b.length != 6) b = new double[]{-1, -1, -1, -1, -1, -1};
        
        SmartDashboard.putNumber("campose 1", b[0]);
        SmartDashboard.putNumber("campose 2", b[1]);
        SmartDashboard.putNumber("campose 3", b[2]);
        SmartDashboard.putNumber("campose 4", b[3]);
        SmartDashboard.putNumber("campose 5", b[4]);
        SmartDashboard.putNumber("campose 6", b[5]);
        
        // var m = table.getAllEntries();
        //for(var i : m.entrySet()){
            //SmartDashboard.putString("limelight-" + i.getKey(), i.getValue());
        //}
    }
    public static double getZDistance() {
        var arr = Network.getTable("limelight").getDoubleArray("botpose");
        if(arr.length == 0) {
            return Double.NaN;
        }
        return Network.getTable("limelight").getDoubleArray("botpose")[0];
    }
    public static double getXDistance() {
        var arr = Network.getTable("limelight").getDoubleArray("botpose");
        if(arr.length == 0) {
            return Double.NaN;
        }
        return Network.getTable("limelight").getDoubleArray("botpose")[1];
    }
    public static double getYDistance() {
        var arr = Network.getTable("limelight").getDoubleArray("botpose");
        if(arr.length == 0) {
            return Double.NaN;
        }
        return Network.getTable("limelight").getDoubleArray("botpose")[2];
    }
    public static double getZRotation() {
        var arr = Network.getTable("limelight").getDoubleArray("botpose");
        if(arr.length == 0) {
            return Double.NaN;
        }
        return Network.getTable("limelight").getDoubleArray("botpose")[3];
    }
    public static double getXRotation() {
        var arr = Network.getTable("limelight").getDoubleArray("botpose");
        if(arr.length == 0) {
            return Double.NaN;
        }
        return Network.getTable("limelight").getDoubleArray("botpose")[4];
    }
    public static double getYRotation() {
        var arr = Network.getTable("limelight").getDoubleArray("botpose");
        if(arr.length == 0) {
            return Double.NaN;
        }
        return Network.getTable("limelight").getDoubleArray("botpose")[5];
    }
}

