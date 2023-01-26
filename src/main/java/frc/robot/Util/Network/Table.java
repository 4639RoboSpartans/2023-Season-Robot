package frc.robot.Util.Network;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Table {
    private final String name;
    // The networktable for the limelight
    private final NetworkTable table;
    // Cache the NetworkTableEntries so we don't have to keep accessing it
    private final Map<String, NetworkTableEntry> entryCache = new HashMap<>();

    protected Table(String name, NetworkTable table){
        this.name = name;
        this.table = table;
    }

    public String getName() {
        return name;
    }

    private NetworkTableEntry getEntry(String key){
        if(entryCache.containsKey(key))
            return entryCache.get(key);
        var entry = table.getEntry(key);
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
