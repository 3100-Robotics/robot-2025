package frc.robot.subsystems;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Algae extends SubsystemBase {
    private final int motorID = 19;

    private SparkBaseConfig config = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(30, 60);

    private SparkMax motor = new SparkMax(motorID, MotorType.kBrushless);

    private DigitalInput limitSwitch = new DigitalInput(0);

    private Trigger currentTrigger; 
    private Trigger limitTrigger;
    private LinearFilter currentFilter = LinearFilter.movingAverage(30);
    
    public Algae() {
        motor.configure(config, null, null);
        currentTrigger = new Trigger(() -> currentFilter.calculate(motor.getOutputCurrent()) >= 15);
        limitTrigger = new Trigger(() -> !limitSwitch.get());
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("motor current", currentFilter.calculate(motor.getOutputCurrent()));
        SmartDashboard.putBoolean("limit switch", limitTrigger.getAsBoolean());
    }

    public Command set(double speed) {
        return runOnce(() -> motor.set(speed)); // sets the moters to run at @speed 
    }

    public Trigger limitHit() {
        // return distanceTrigger.and(currentTrigger);
        return limitTrigger;
    }

    public Trigger currentHit() {
        return currentTrigger;
    }
}
