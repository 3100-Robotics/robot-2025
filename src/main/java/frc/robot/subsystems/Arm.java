package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Arm implements Subsystem{
    private final int pivotMotorID = 0;
    private final int pivotEncoderID = 0;

    private CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration().
        withMagnetSensor(new MagnetSensorConfigs()
            .withMagnetOffset(0)
            .withSensorDirection(SensorDirectionValue.Clockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(0));

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
            .withReverseSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(0)
            .withForwardSoftLimitEnable(false));


    private TalonFX pivotMotor = new TalonFX(pivotMotorID);

    private CANcoder pivotEncoder = new CANcoder(pivotEncoderID);
    
    private double setpoint;
    private final Trigger atSetpoint;

    public Arm() {
        pivotEncoder.getConfigurator().apply(encoderConfiguration);

        motorConfigs.withFeedback(new FeedbackConfigs()
            .withRemoteCANcoder(pivotEncoder)
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(1));
        pivotMotor.getConfigurator().apply(motorConfigs);

        atSetpoint = new Trigger(() -> Math.abs(pivotMotor.getPosition().getValueAsDouble() - setpoint) < 0.005);
    }

    public Command set(double speed) {
        return this.run(() -> pivotMotor.set(speed));
    }

    public Command goToPos(double pos, BooleanSupplier shouldLimitForwardMotion, BooleanSupplier shouldLimitBackwardMotion) {
        return this.run(() -> {
            setpoint = pos;
            pivotMotor.setControl(new MotionMagicExpoVoltage(pos)
                .withLimitForwardMotion(shouldLimitForwardMotion.getAsBoolean())
                .withLimitReverseMotion(shouldLimitBackwardMotion.getAsBoolean()));})
            .until(atSetpoint);
    }

    public Trigger atSetpoint() {
        return atSetpoint;
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

}
