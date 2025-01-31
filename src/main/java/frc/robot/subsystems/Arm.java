package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.configs.AudioConfigs;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
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

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Arm extends SubsystemBase {
    private final int pivotMotorID = 16;
    private final int pivotEncoderID = 17;

    private final double gearRatio = 56.78;
    private final double armLength = 0.69;

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
            .withKP(10)
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
            .withForwardSoftLimitEnable(false))
        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicAcceleration(10));

// declaration of motors and CANcoders

    private TalonFX pivotMotor = new TalonFX(pivotMotorID);

    private CANcoder pivotEncoder = new CANcoder(pivotEncoderID);
    
    private double setpoint;
    private final Trigger atSetpoint;

// arm sim set up
    private SingleJointedArmSim armSim = new SingleJointedArmSim( 
        DCMotor.getKrakenX60(1), gearRatio, 1.086414471, armLength, // sets arm motors, gear ratios, the momentum of the arm, and the arm length
        -Math.PI/2, Math.PI/2, true, 0); // sets the min and max angles of the arm, sets simulation gravity, and the starting angle

    public Arm() {
        pivotEncoder.getConfigurator().apply(encoderConfiguration);

        motorConfigs.withFeedback(new FeedbackConfigs()
            .withRemoteCANcoder(pivotEncoder)
            .withRotorToSensorRatio(1)
            .withSensorToMechanismRatio(1));
        pivotMotor.getConfigurator().apply(motorConfigs);

        atSetpoint = new Trigger(() -> Math.abs(pivotMotor.getPosition().getValueAsDouble() - setpoint) < 0.005);
    }

    @Override
    public void simulationPeriodic() {
        var talonFXSim = pivotMotor.getSimState();
        var encoderSim = pivotEncoder.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        armSim.setInputVoltage(motorVoltage.in(Volts));
        armSim.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        encoderSim.setRawPosition(armSim.getAngleRads()/(2*Math.PI));
        encoderSim.setVelocity(armSim.getVelocityRadPerSec()/(2*Math.PI));

        talonFXSim.setRawRotorPosition(armSim.getAngleRads()*gearRatio/(2*Math.PI));
        talonFXSim.setRotorVelocity(armSim.getVelocityRadPerSec()*gearRatio/(2*Math.PI));
    }

    public Command set(double speed) {
        return this.run(() -> pivotMotor.set(speed));
    }

    public Command goToPos(double pos) {
        return this.run(() -> {
            // System.out.println("arm has been told to move!");
            setpoint = pos;
            pivotMotor.setControl(new MotionMagicExpoVoltage(pos));})
            .until(atSetpoint);
    }

    public Trigger atSetpoint() {
        return atSetpoint;
    }

    public double getPosition() {
        return pivotMotor.getPosition().getValueAsDouble();
    }

}
