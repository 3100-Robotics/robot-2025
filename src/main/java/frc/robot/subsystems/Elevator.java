package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Trigger;


public class Elevator implements Subsystem{
    private final int elevatorMotor1ID = 13;
    private final int elevatorMotor2ID = 14;

    private final double upperSoftLimit = 0;
    
    private final double gearRatio = 7.75;

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
            .withKP(0.1)
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
            .withForwardSoftLimitEnable(true))
        .withFeedback(new FeedbackConfigs()
            .withSensorToMechanismRatio(gearRatio*0.0444))
        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(10));


    private TalonFX elevatorMotor1 = new TalonFX(elevatorMotor1ID);
    private TalonFX elevatorMotor2 = new TalonFX(elevatorMotor2ID);
    
    private double setpoint;
    private final Trigger atSetpoint;

    private final DCMotorSim m_motorSimModel = new DCMotorSim(
        LinearSystemId.createDCMotorSystem(
            DCMotor.getKrakenX60(2), 0.001, gearRatio),
        DCMotor.getKrakenX60(2));

    public Elevator() {
        elevatorMotor1.getConfigurator().apply(motorConfigs);
        elevatorMotor2.getConfigurator().apply(motorConfigs);
        elevatorMotor2.setControl(new Follower(elevatorMotor1ID, false));

        atSetpoint = new Trigger(() -> Math.abs(elevatorMotor1.getPosition().getValueAsDouble() - setpoint) < 0.005);
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = elevatorMotor1.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        m_motorSimModel.setInputVoltage(motorVoltage.in(Volts));
        m_motorSimModel.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        talonFXSim.setRawRotorPosition(m_motorSimModel.getAngularPosition().times(gearRatio));
        talonFXSim.setRotorVelocity(m_motorSimModel.getAngularVelocity().times(gearRatio));
    }

    public Command set(double speed) {
        return this.run(() -> elevatorMotor1.set(speed));
    }

    public Command goToPos(double pos) {
        return this.runOnce(() -> {
            System.out.println("elevator needs to move!");
            setpoint = pos;
            elevatorMotor1.setControl(new MotionMagicExpoVoltage(pos));})
            .until(atSetpoint);
    }

    public Trigger atSetpoint() {
        return atSetpoint;
    }

    public double getHeight() {
        return elevatorMotor1.getPosition().getValueAsDouble();
    }
}
