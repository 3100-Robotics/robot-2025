package frc.robot.subsystems;


import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Climber implements Subsystem {
    private final int motorID = 0;

    private SparkBaseConfig config = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .inverted(false)
        .smartCurrentLimit(40, 40);

    private SparkMax winch = new SparkMax(motorID, MotorType.kBrushless);

    private PIDController angleController = new PIDController(0.3, 0, 0);

    public Climber () {
        winch.configure(config, null, null);
    }
    public Command setSpeed(double speed) {
        return runOnce(() -> winch.set(speed));

    }

    public Command goToPos(double pos) {
        return run(() -> {
            angleController.setSetpoint(pos);
            winch.setVoltage(angleController.calculate(winch.getEncoder().getPosition()));
        }).until(angleController::atSetpoint);
    }
}