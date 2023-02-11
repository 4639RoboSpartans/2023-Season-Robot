package frc.robot.Util.Network;

import java.util.HashMap;
import java.util.Map;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Network {
    // Implement the singleton pattern
    private static Network instance = null;
    private static Network getInstance(){
        if(instance == null)
            instance = new Network();
        return instance;
    }
    
    // The main object we use to interact with the NetworkTables
    private final NetworkTableInstance tableInstance;

    private final Map<String, Table> tableCache = new HashMap<>();

    private Network(){
        tableInstance = NetworkTableInstance.getDefault();
    }

    private Table _getTable(String tableName){
        if(tableCache.containsKey(tableName)){
            return tableCache.get(tableName);
        }
        var res = new Table(tableName, tableInstance.getTable(tableName));
        tableCache.put(tableName, res);
        return res;
    }

    public static Table getTable(String tableName){
        return getInstance()._getTable(tableName);
    }
}