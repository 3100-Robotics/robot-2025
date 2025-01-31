package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Coral implements Subsystem {
    private final int motorID = 18;

    private SparkBaseConfig config = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(30, 40);

    private SparkMax motor = new SparkMax(motorID, MotorType.kBrushless);

    private Trigger currentTrigger;
    private LinearFilter currentFilter = LinearFilter.movingAverage(20);
    
    public Coral() {
        motor.configure(config, null, null);
        currentTrigger = new Trigger(() -> currentFilter.calculate(motor.getOutputCurrent()) >= 20);
    }

    public Command set(double speed) {
        return runOnce(() -> motor.set(speed));
    }

    public Trigger currentHit() {
        return currentTrigger;
    }
}
