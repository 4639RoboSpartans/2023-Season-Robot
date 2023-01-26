package frc.robot.Vision;

import frc.robot.Util.Network.Network;
import frc.robot.Util.Network.Table;

public class LimeLight {
    private static Table table;
    public static Table getTable() {
        if(table == null)
            table = Network.getInstance().getTable("limelight");
        return table;
    }
}