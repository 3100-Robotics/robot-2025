package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Subsystem;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Algae implements Subsystem {
    private final int thing1 = 9999;

    private SparkBaseConfig config = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(30, 40);

    private SparkMax motor = new SparkMax(thing1, MotorType.kBrushless);
    
    public Algae() {
        
    }
}
