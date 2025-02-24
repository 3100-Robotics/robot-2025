package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.util.Optional;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.RangingMode;

public class Algae extends SubsystemBase {
    private final int motorID = 19;
    private final int laserCanID = 18;

    private SparkBaseConfig config = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(30, 60);

    private SparkMax motor = new SparkMax(motorID, MotorType.kBrushless);

    private LaserCan laserCan = new LaserCan(laserCanID);

    private Trigger currentTrigger; 
    private Trigger distanceTrigger;
    private LinearFilter currentFilter = LinearFilter.movingAverage(40);
    
    public Algae() {
        motor.configure(config, null, null);
        currentTrigger = new Trigger(() -> currentFilter.calculate(motor.getOutputCurrent()) >= 20);

        try {
            laserCan.setRangingMode(RangingMode.SHORT);
        }
        catch (ConfigurationFailedException e) {
            System.out.println("laser can range setting failed");
        }

        distanceTrigger = new Trigger(() -> {
            Optional<Integer> distance = Optional.ofNullable((Integer) laserCan.getMeasurement().distance_mm);
            
            if (distance.isPresent()) {
                return distance.get()==0 ? false : distance.get() <= 60;
            }
            return false;
        });
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("motor current", currentFilter.calculate(motor.getOutputCurrent()));
    }

    public Command set(double speed) {
        return runOnce(() -> motor.set(speed)); // sets the moters to run at @speed 
    }

    public Trigger currentHit() {
        return distanceTrigger.and(currentTrigger);
    }
}
