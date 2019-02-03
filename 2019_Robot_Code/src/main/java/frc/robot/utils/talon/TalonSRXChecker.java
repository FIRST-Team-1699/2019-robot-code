package frc.robot.utils.talon;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.Subsystem;
import frc.robot.utils.Util;

import java.util.ArrayList;
import java.util.function.DoubleSupplier;

public class TalonSRXChecker {
    public static class CheckerConfig {
        public double currentFloor = 5;
        public double rpmFloor = 2000;

        public double currentEpsilon = 5.0;
        public double rpmEpsilon = 500;
        public DoubleSupplier rpmSupplier = null;

        public double runTimeSec = 4.0;
        public double waitTimeSec = 2.0;
        public double runOutputPercentage = 0.5;
    }

    public static class TalonSRXConfig {
        public String name;
        public TalonSRX talon;

        public TalonSRXConfig(String name, TalonSRX talon){
            name = name;
            talon = talon;
        }
    }

    private static class StoredTalonSRXConfiguration {
        public ControlMode mode;
        public double setValue;
    }

    public static boolean CheckTalons(Subsystem subsystem, ArrayList<TalonSRXConfig> talonsToCheck, CheckerConfig checkerConfig){
        boolean failure = false;
        System.out.println("/////////////////////////////////////////////");
        System.out.println("Checking subsystem " + subsystem.getClass() + " for " + talonsToCheck.size() + " talons.");

        ArrayList<Double> currents = new ArrayList<>();
        ArrayList<Double> rpms = new ArrayList<>();
        ArrayList<StoredTalonSRXConfiguration> storedConfigurations = new ArrayList<>();
        
        for(TalonSRXConfig config : talonsToCheck){
            LazyTalonSRX talon = LazyTalonSRX.class.cast(config.talon);
            
            StoredTalonSRXConfiguration configuration = new StoredTalonSRXConfiguration();
            configuration.mode = talon.getControlMode();
            configuration.setValue = talon.getLastSet();

            storedConfigurations.add(configuration);

            talon.set(ControlMode.PercentOutput, 0.0);
        }

        for(TalonSRXConfig config : talonsToCheck){
            System.out.println("Checking: " + config.name);

            config.talon.set(ControlMode.PercentOutput, checkerConfig.runOutputPercentage);
            Timer.delay(checkerConfig.runTimeSec);

            double current = config.talon.getOutputCurrent();
            currents.add(current);
            System.out.println("Current: " + current);

            double rpm = Double.NaN;
            if(checkerConfig.rpmSupplier != null){
                rpm = checkerConfig.rpmSupplier.getAsDouble();
                rpms.add(rpm);
                System.out.println("RPM: " + rpm);
            }
            System.out.println('\n');

            config.talon.set(ControlMode.PercentOutput, 0.0);

            if(current < checkerConfig.currentFloor){
                System.out.println(config.name + " has failed current floor check vs " + checkerConfig.currentFloor + "!!");
                failure = true;
            }
            if(checkerConfig.rpmSupplier != null){
                if(rpm < checkerConfig.rpmFloor){
                    System.out.println(config.name + " has failed rpm check vs " + checkerConfig.rpmFloor + "!!");
                    failure = true;
                }
            }
            Timer.delay(checkerConfig.waitTimeSec);
        }

        if(currents.size() > 0){
            Double average = currents.stream().mapToDouble(val -> val).average().getAsDouble();

            if(!Util.allCloseTo(currents, average, checkerConfig.currentEpsilon)){
                System.out.println("RPMs varied!!!!!!!!!!!!!");
                failure = true;
            }
        }

        for(int i = 0; i < talonsToCheck.size(); ++i){
            talonsToCheck.get(i).talon.set(storedConfigurations.get(i).mode, storedConfigurations.get(i).setValue);
        }

        return !failure;
    }
}
