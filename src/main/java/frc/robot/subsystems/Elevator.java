package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class Elevator implements Subsystem{
    private final int elevatorMotor1ID = 0;
    private final int elevatorMotor2ID = 0;

    private final double upperSoftLimit = 0;

    private TalonFXConfiguration motorConfigs = new TalonFXConfiguration().
        withAudio(new AudioConfigs()
            .withBeepOnBoot(true)
            .withBeepOnConfig(true)).
        withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.Clockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake))
        .withSlot0(new Slot0Configs()
            .withKP(0)
            .withKI(0)
            .withKD(0)
            .withKG(0)
            .withKS(0)
            .withKV(0)
            .withKA(0)
            .withGravityType(GravityTypeValue.Elevator_Static))
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitThreshold(0)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(upperSoftLimit)
            .withForwardSoftLimitEnable(true));


    private TalonFX elevatorMotor1 = new TalonFX(elevatorMotor1ID);
    private TalonFX elevatorMotor2 = new TalonFX(elevatorMotor2ID);
    
    private double setpoint;
    private final Trigger atSetpoint;

    public Elevator() {
        elevatorMotor1.getConfigurator().apply(motorConfigs);
        elevatorMotor2.getConfigurator().apply(motorConfigs);
        elevatorMotor2.setControl(new Follower(elevatorMotor1ID, false));

        atSetpoint = new Trigger(() -> Math.abs(elevatorMotor1.getPosition().getValueAsDouble() - setpoint) < 0.005);
    }

    public Command set(double speed) {
        return this.run(() -> elevatorMotor1.set(speed));
    }

    public Command goToPos(double pos, BooleanSupplier shouldLimitForwardMotion, BooleanSupplier shouldLimitBackwardMotion) {
        return this.runOnce(() -> {
            setpoint = pos;
            elevatorMotor1.setControl(new MotionMagicExpoVoltage(pos)
                .withLimitForwardMotion(shouldLimitForwardMotion.getAsBoolean())
                .withLimitReverseMotion(shouldLimitBackwardMotion.getAsBoolean()));})
            .until(atSetpoint);
    }

    public Trigger atSetpoint() {
        return atSetpoint;
    }

    public double getHeight() {
        return elevatorMotor1.getPosition().getValueAsDouble();
    }
}
