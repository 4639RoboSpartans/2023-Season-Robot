package frc.robot.Util.Network;

import edu.wpi.first.networktables.NetworkTableInstance;

public class Network {
    // The main object we use to interact with the NetworkTables
    private NetworkTableInstance tableInstance;

    // Implement the singleton pattern
    private static Network instance = null;
    public static Network getInstance(){
        if(instance == null)
            instance = new Network();
        return instance;
    }

    private Network(){
        tableInstance = NetworkTableInstance.getDefault();
    }

    public Table getTable(String tableName){
        return new Table(tableName, tableInstance.getTable(tableName));
    }
}