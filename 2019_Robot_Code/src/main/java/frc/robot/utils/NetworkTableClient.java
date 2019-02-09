package frc.robot.utils;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.constants.Constants;

import java.util.HashMap;
import java.util.Map;

/*
* Use for reference:
* https://wpilib.screenstepslive.com/s/currentCS/m/75361/l/851714-creating-a-client-side-program
*/

//TODO Update or remove
public class NetworkTableClient implements Runnable{

    //Code for singleton
    private static NetworkTableClient instance;

    public static NetworkTableClient getInstance(){
        if(instance == null){
            instance = new NetworkTableClient();
        }
        return instance;
    }

    //Thread vars
    private Thread thread;
    private boolean running;
    private final Map<String, NetworkTableEntry> ntMap;

    private NetworkTableClient(){
        ntMap = new HashMap<>();
    }

    @Override
    public void run() {
        //Start NetworkTable
        NetworkTableInstance ntInstance = NetworkTableInstance.getDefault();
        NetworkTable nt = ntInstance.getTable("datatable"); //TODO Change to correct name
        ntInstance.startClient(Constants.teamNumber);
        ntInstance.startDSClient();

        //NetworkTable Entries
        //TODO Add entries
        while(running){
            //TODO Populate
            //Loop keys in ntMap, and assign to TableEntry
            for(String key : ntMap.keySet()){
                synchronized (ntMap){
                    ntMap.put(key, ntInstance.getEntry(key));
                }
            }
        }
    }

    //Map methods
    public void addEntryKey(final String key){
        ntMap.put(key, null);
    }

    public synchronized NetworkTableEntry getEntry(final String key){
        return ntMap.get(key);
    }

    //Used to start/stop the threads
    public synchronized void start(){
        if(running){
            return;
        }
        thread = new Thread(this);
        running = true;
        thread.start();
    }

    public synchronized void stop(){
        if(!running){
            return;
        }
        running = false;
        try {
            thread.join();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }
}
