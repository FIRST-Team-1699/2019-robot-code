package frc.robot.utils;

import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.Date;
import java.util.UUID;

public class CrashTracker {

    private static final UUID RUN_INSTANCE_UUID = UUID.randomUUID();

    //TODO Add log methods
    public static void logThrowableCrash(Throwable throwable){
        logMarker("Exception", throwable);
    }

    private static void logMarker(String mark){
        logMarker(mark, null);
    }

    private static void logMarker(String mark, Throwable nullableException){
        try(PrintWriter writer = new PrintWriter(new FileWriter("/home/lvuser/crash_tracking.txt", true))){
            writer.print(RUN_INSTANCE_UUID.toString());
            writer.print(", ");
            writer.print(mark);
            writer.print(", ");
            writer.print(new Date().toString());

            if(nullableException != null){
                writer.print(", ");
                nullableException.printStackTrace();
            }

            writer.println();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
