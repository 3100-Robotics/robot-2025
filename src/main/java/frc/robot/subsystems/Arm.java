package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
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
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class Arm extends SubsystemBase {
    private final int pivotMotorID = 16;
    private final int pivotEncoderID = 17;

    private final double gearRatio = 56.78;
    private final double armLength = 0.69;

    private CANcoderConfiguration encoderConfiguration = new CANcoderConfiguration().
        withMagnetSensor(new MagnetSensorConfigs()
            .withMagnetOffset(-0.004639)
            .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
            .withAbsoluteSensorDiscontinuityPoint(1));

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
            .withKP(30)
            .withKG(0.42)
            .withKD(0.2)
            // .withKP(63.093) // 10
            // .withKI(0)
            // .withKD(30.827)
            // .withKG(0.72337)
            // .withKS(0)
            // .withKV(7.181)
            // .withKA(9.4905)
            .withGravityType(GravityTypeValue.Arm_Cosine))
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitThreshold(0)
            .withReverseSoftLimitEnable(false)
            .withForwardSoftLimitThreshold(0)
            .withForwardSoftLimitEnable(false))
        .withMotionMagic(new MotionMagicConfigs());
            // .withMotionMagicCruiseVelocity(6));


    private TalonFX pivotMotor = new TalonFX(pivotMotorID);

    private CANcoder pivotEncoder = new CANcoder(pivotEncoderID);
    
    private double setpoint;
    private final Trigger atSetpoint;

    private final SysIdRoutine sysid = new SysIdRoutine(new SysIdRoutine.Config(
            null,
            Volts.of(1),
            null,
            state -> SignalLogger.writeString("arm sysid state", state.toString())),
            new SysIdRoutine.Mechanism(
                    voltage -> pivotMotor.setVoltage(voltage.baseUnitMagnitude()),
                    null,
                    this));

    private SingleJointedArmSim armSim = new SingleJointedArmSim(
        DCMotor.getKrakenX60(1), gearRatio, 1.086414471, armLength,
        -3*Math.PI/2, 3*Math.PI/2, true, 0);

    public Arm() {
        pivotEncoder.getConfigurator().apply(encoderConfiguration);

        motorConfigs.withFeedback(new FeedbackConfigs()
            .withRemoteCANcoder(pivotEncoder)
            .withRotorToSensorRatio(1/gearRatio)
            .withSensorToMechanismRatio(1));
        pivotMotor.getConfigurator().apply(motorConfigs);

        atSetpoint = new Trigger(() -> Math.abs(pivotMotor.getPosition().getValueAsDouble() - setpoint) < 0.005);
        // pivotEncoder.setPosition(0);
        // pivotMotor.setPosition(0);

        if (Utils.isSimulation()) {
//            pivotMotor.setPosition(0.25);
//            pivotEncoder.setPosition(0.25);
//            pivotEncoder.getSimState().setRawPosition(0.25);
//            armSim.setState(Math.PI/2, 0);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("arm setpoint", setpoint);
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

    public Command sysidDynamic(SysIdRoutine.Direction direction) {
        return sysid.dynamic(direction);
    }

    public Command sysidQuasistatic(SysIdRoutine.Direction direction) {
        return sysid.quasistatic(direction);
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
