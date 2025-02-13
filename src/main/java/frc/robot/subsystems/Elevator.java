package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.SignalLogger;
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
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;


public class Elevator extends SubsystemBase {
    private final int elevatorMotor1ID = 14;
    private final int elevatorMotor2ID = 15;

    private final double upperSoftLimit = 0;
    
    private final double gearRatio = 7.75;
    private final double sprocketRadius = 0.0444/2;
    private final double carriageRatio = 3;

    private TalonFXConfiguration motorConfigs = new TalonFXConfiguration().
        withAudio(new AudioConfigs()
            .withBeepOnBoot(true)
            .withBeepOnConfig(true)).
        withCurrentLimits(new CurrentLimitsConfigs()
            .withSupplyCurrentLimit(40)
            .withSupplyCurrentLimitEnable(true))
        .withMotorOutput(new MotorOutputConfigs()
            .withInverted(InvertedValue.CounterClockwise_Positive)
            .withNeutralMode(NeutralModeValue.Brake))
        .withSlot0(new Slot0Configs()
            .withKP(20)
            .withKG(0.35)
            // .withKP(61.207) // 61.207
            // .withKI(0)
            // .withKD(13.152) // 13.152
            // .withKG(0.59319) // 0.59319
            // .withKS(0.019567) // 0.019567
            // .withKV(7.2027) // 7.2027
            // .withKA(1.2414) // 1.2414
            .withGravityType(GravityTypeValue.Elevator_Static))
        .withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
            .withReverseSoftLimitThreshold(0)
            .withReverseSoftLimitEnable(true)
            .withForwardSoftLimitThreshold(upperSoftLimit)
            .withForwardSoftLimitEnable(false))
        .withFeedback(new FeedbackConfigs()
            .withSensorToMechanismRatio(gearRatio))
        .withMotionMagic(new MotionMagicConfigs()
            .withMotionMagicCruiseVelocity(2));


    private TalonFX elevatorMotor1 = new TalonFX(elevatorMotor1ID);
    private TalonFX elevatorMotor2 = new TalonFX(elevatorMotor2ID);
    
    private double setpoint;
    private final Trigger atSetpoint;

    private final SysIdRoutine sysid = new SysIdRoutine(new SysIdRoutine.Config(
            Volts.of(0.5).per(Second),
            Volts.of(3),
            null,
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())),
            new SysIdRoutine.Mechanism(
                    voltage -> elevatorMotor1.setVoltage(voltage.baseUnitMagnitude()),
                    null,
                    this));

    private final ElevatorSim elevatorSim = new ElevatorSim(   
        DCMotor.getKrakenX60(2),
        gearRatio,
        10, sprocketRadius, 0, 1.87, true, 0);

    public Elevator() {
        elevatorMotor1.getConfigurator().apply(motorConfigs);
        elevatorMotor2.getConfigurator().apply(motorConfigs);
        elevatorMotor2.setControl(new Follower(elevatorMotor1ID, false));

        atSetpoint = new Trigger(() -> Math.abs(elevatorMotor1.getPosition().getValueAsDouble() - setpoint) < 0.1);

        // if (Utils.isSimulation()) {
        //     elevatorMotor1.getSimState().Orientation = ChassisReference.Clockwise_Positive;
        // }
    }

    @Override
    public void periodic() {
        elevatorMotor2.setControl(new Follower(elevatorMotor1ID, false));

        SmartDashboard.putNumber("elevator setpoint", setpoint);
        SmartDashboard.putNumber("elevator1 pos", elevatorMotor1.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator2 pos", elevatorMotor2.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("elevator1 output", elevatorMotor1.getMotorVoltage().getValueAsDouble());
        SmartDashboard.putNumber("elevator2 output", elevatorMotor2.getMotorVoltage().getValueAsDouble());
    }

    @Override
    public void simulationPeriodic() {
        // System.out.println(setpoint);`
        var talonFXSim = elevatorMotor1.getSimState();

        // set the supply voltage of the TalonFX
        talonFXSim.setSupplyVoltage(RobotController.getBatteryVoltage());

        // get the motor voltage of the TalonFX
        var motorVoltage = talonFXSim.getMotorVoltageMeasure();

        // use the motor voltage to calculate new position and velocity
        // using WPILib's DCMotorSim class for physics simulation
        elevatorSim.setInputVoltage(motorVoltage.in(Volts));
        elevatorSim.update(0.020); // assume 20 ms loop time

        // apply the new rotor position and velocity to the TalonFX;
        // note that this is rotor position/velocity (before gear ratio), but
        // DCMotorSim returns mechanism position/velocity (after gear ratio)
        System.out.println(elevatorSim.getPositionMeters());
        talonFXSim.setRawRotorPosition(elevatorSim.getPositionMeters()*gearRatio* carriageRatio /(sprocketRadius *2*Math.PI));
        talonFXSim.setRotorVelocity(elevatorSim.getVelocityMetersPerSecond()*gearRatio* carriageRatio /(sprocketRadius *2*Math.PI));
    }

    public Command sysidDynamic(SysIdRoutine.Direction direction) {
        return sysid.dynamic(direction);
    }

    public Command sysidQuasistatic(SysIdRoutine.Direction direction) {
        return sysid.quasistatic(direction);
    }

    public Command set(double speed) {
        return this.run(() -> elevatorMotor1.set(speed));
    }

    public Command goToPos(double pos) {
        return this.run(() -> {
            double rotations = pos / (Math.PI * 2 * sprocketRadius * carriageRatio);
            setpoint = rotations;
            elevatorMotor1.setControl(new MotionMagicExpoVoltage(rotations));})
            .until(atSetpoint);
    }

    public Trigger atSetpoint() {
        return atSetpoint;
    }

    public double getHeight() {
        return elevatorMotor1.getPosition().getValueAsDouble()* sprocketRadius *2*Math.PI;
    }
}
