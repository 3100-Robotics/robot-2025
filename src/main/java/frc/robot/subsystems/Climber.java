package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.function.DoubleSupplier;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

public class Climber extends SubsystemBase {
    private final int motorID = 20;

    private SparkBaseConfig config = new SparkMaxConfig()
        .idleMode(IdleMode.kBrake)
        .inverted(true)
        .smartCurrentLimit(40, 40);

    private SparkMax winch = new SparkMax(motorID, MotorType.kBrushless);

    private PIDController angleController = new PIDController(0.3, 0, 0);

    public Climber () {
        winch.configure(config, null, null);
    }

    public Command setSpeed(DoubleSupplier speed) {
        return run(() -> winch.set(MathUtil.applyDeadband(speed.getAsDouble(), 0.07)));

    }

    public Command goToPos(double pos) {
        return run(() -> {
            angleController.setSetpoint(pos*25);
            winch.setVoltage(angleController.calculate(winch.getEncoder().getPosition()));
        }).until(angleController::atSetpoint);
    }
}
