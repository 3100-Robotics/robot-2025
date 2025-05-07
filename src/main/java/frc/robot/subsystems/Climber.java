package frc.robot.subsystems;


import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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

    private PIDController angleController = new PIDController(0.6, 0, 0); // 0.3

    private Trigger atSetpoint;

    public Climber () {
        winch.configure(config, null, null);
        angleController.setTolerance(1);
        atSetpoint = new Trigger(angleController::atSetpoint);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("climper pos", winch.getEncoder().getPosition());
    }

    public Command setSpeed(DoubleSupplier speed) {
        return run(() -> winch.set(MathUtil.applyDeadband(speed.getAsDouble(), 0.07)));

    }

    public Command goToPos(double pos) {
        return runOnce(() -> angleController.setSetpoint(pos*25)).andThen(run(() -> {
            double speed = angleController.calculate(winch.getEncoder().getPosition());
            SmartDashboard.putNumber("climber output", speed);
            winch.setVoltage(
                speed + 0.5);
        }));
    }

    public Trigger atSetpoint() {
        return atSetpoint;
    }
}
