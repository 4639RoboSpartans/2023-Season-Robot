package frc.robot.Vision;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class LimeLight {
    private static LimeLight instance = null;
    public static LimeLight getInstance(){
        return instance == null ? instance = new LimeLight() : instance;
    }

    private NetworkTableInstance tableInstance;
    private NetworkTable limelightTable;
    private Map<String, NetworkTableEntry> entryCache = new HashMap<>();
    private LimeLight(){
        tableInstance = NetworkTableInstance.getDefault();
        limelightTable = tableInstance.getTable("limelight");

    }

    private NetworkTableEntry getEntry(String key){
        if(entryCache.containsKey(key))
            return entryCache.get(key);
        var entry = limelightTable.getEntry(key);
        entryCache.put(key, entry);
        return entry;
    }

    public double getNumber(String key){
        return (double) getEntry(key).getNumber(0.0);
    }

    public int getInteger(String key){
        return (int) getEntry(key).getInteger(0);
    }

    public String getString(String key){
        return getEntry(key).getString("");
    }
}