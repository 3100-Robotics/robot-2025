// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.States;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Climber climber = new Climber();
    public final Algae algae = new Algae();

    public final Elevator elevator = new Elevator();
    public final Arm arm = new Arm();
    public final Superstructure superstructure = new Superstructure(elevator, arm);

    public final Vision vision = new Vision(drivetrain::getPos, drivetrain.getField());

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController coDriverJoystick = new CommandXboxController(1);

    public final AutoChooser autoSelector = new AutoChooser();

    private final AutoFactory autoFactory;

    public RobotContainer() {
        autoFactory = new AutoFactory(
                () -> drivetrain.getState().Pose, // A function that returns the current robot pose
                drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
                drivetrain::followTrajectory, // The drive subsystem trajectory follower
                true, // If alliance flipping should be enabled
                drivetrain // The drive subsystem
        );

        configureBindings();
        // configureSysidBindings();
        configureAutonomous();
    }

    public Command collectAlgae(States collectingState, String side) {
        return Commands.sequence(
                algae.set(-1),
                superstructure.goToPos(collectingState, side).until(algae.currentHit()),
                Commands.waitUntil(algae.currentHit()),
                Commands.waitSeconds(0.15),
                superstructure.goToPos(States.resting, "neither"),
                algae.set(0));
    }

    public Command scoreAlgae(States scoringPos, String side) {
        return Commands.sequence(
                superstructure.goToPos(scoringPos, side),
                algae.set(-0.5),
                Commands.waitSeconds(0.1),
                algae.set(0.5),
                Commands.waitSeconds(0.5),
                algae.set(0),
                superstructure.goToPos(States.resting, "neither"));
    }

    public AutoRoutine leave() {
        AutoRoutine routine = autoFactory.newRoutine("leave");

        AutoTrajectory leaveTraj = routine.trajectory("leave");

        routine.active().onTrue(
                Commands.sequence(
                        leaveTraj.resetOdometry(),
                        leaveTraj.cmd()
                )
        );

        return routine;
    }

    public AutoRoutine score1Algae() {
        AutoRoutine routine = autoFactory.newRoutine("1 algae");

        AutoTrajectory startToAlgae1 = routine.trajectory("start-algae1");
        AutoTrajectory algae1ToScore = routine.trajectory("algae1-score");

        routine.active().onTrue(Commands.sequence(
                startToAlgae1.resetOdometry(),
                startToAlgae1.cmd()));

        startToAlgae1.done().onTrue(Commands.sequence(
                collectAlgae(States.algaeFromReefLow, "left"),
                algae1ToScore.spawnCmd()));

        algae1ToScore.done().onTrue(Commands.sequence(
                scoreAlgae(States.algaeToBardge, "right")));

        return routine;
    }

    public AutoRoutine score2Algae() {
        AutoRoutine algae1 = score1Algae();

        AutoRoutine routine = autoFactory.newRoutine("2 algae");

        AutoTrajectory scoreToAlgae2 = routine.trajectory("score-algae2");
        AutoTrajectory algae2ToScore = routine.trajectory("algae2-score");

        routine.active().onTrue(Commands.sequence(
            algae1.cmd(),
            scoreToAlgae2.spawnCmd()));

        // algae1.active().onFalse(Commands.sequence(
        //         scoreToAlgae2.spawnCmd()));

        scoreToAlgae2.done().onTrue(Commands.sequence(
                collectAlgae(States.algaeFromReefHigh, "right"),
                algae2ToScore.spawnCmd()));

        algae2ToScore.done().onTrue(scoreAlgae(States.algaeToBardge, "right"));

        return routine;
    }

    private void configureAutonomous() {
        SmartDashboard.putData("auto selector", autoSelector);

        autoSelector.addCmd("nothing", Commands::none);
        autoSelector.addRoutine("leave", this::leave);
        autoSelector.addRoutine("score 1 algae", this::score1Algae);
        autoSelector.addRoutine("score 2 algae", this::score2Algae);
        autoSelector.select("leave");
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-driverJoystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-driverJoystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        ///////////
        // ALGAE //
        ///////////

        driverJoystick.x().onTrue(Commands.sequence(
            algae.set(-1),
            Commands.waitUntil(algae.currentHit()),
            Commands.waitSeconds(0.25),
            algae.set(0)));

        // collection

        // floor
        // left
        coDriverJoystick.a().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromGround, "left"));
        // right
        coDriverJoystick.a().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromGround, "right"));

        // between l2 and l3
        // left
        coDriverJoystick.x().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromReefLow, "left"));
        // right
        coDriverJoystick.x().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromReefLow, "right"));

        // between l3 and l4
        // left
        coDriverJoystick.y().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromReefHigh, "left"));
        // right
        coDriverJoystick.y().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromReefHigh, "right"));

        // scoring

        // processor
        // left
        coDriverJoystick.rightBumper().and(driverJoystick.leftBumper()).onTrue(scoreAlgae(States.algaeToProcessor, "left"));
        // right
        coDriverJoystick.rightBumper().and(driverJoystick.rightBumper()).onTrue(scoreAlgae(States.algaeToProcessor, "right"));

        // barge
        // left
        coDriverJoystick.leftBumper().and(driverJoystick.leftBumper()).onTrue(scoreAlgae(States.algaeToBardge, "left"));
        // right
        coDriverJoystick.leftBumper().and(driverJoystick.rightBumper()).onTrue(scoreAlgae(States.algaeToBardge, "right"));

        coDriverJoystick.b().and(driverJoystick.leftBumper()).onTrue(Commands.sequence(
            algae.set(-1),
            superstructure.goToPos(States.algaeFromLollipop, "left").until(algae.currentHit()),
            Commands.waitUntil(algae.currentHit()),
            Commands.waitSeconds(0.15),
            superstructure.goToPos(States.resting, "neither"),
            algae.set(0)));

        coDriverJoystick.b().and(driverJoystick.rightBumper()).onTrue(Commands.sequence(
            algae.set(-1),
            superstructure.goToPos(States.algaeFromLollipop, "right").until(algae.currentHit()),
            Commands.waitUntil(algae.currentHit()),
            Commands.waitSeconds(0.15),
            superstructure.goToPos(States.resting, "neither"),
            algae.set(0)));

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureSysidBindings() {
        coDriverJoystick.leftBumper().onTrue(arm.goToPos(0.25));
        coDriverJoystick.rightBumper().onTrue(elevator.goToPos(0.75));

        // elevator sysid
        coDriverJoystick.a().whileTrue(elevator.sysidQuasistatic(Direction.kForward));
        coDriverJoystick.b().whileTrue(elevator.sysidQuasistatic(Direction.kReverse));
        coDriverJoystick.x().whileTrue(elevator.sysidDynamic(Direction.kForward));
        coDriverJoystick.y().whileTrue(elevator.sysidDynamic(Direction.kReverse));

        // arm sysid
        coDriverJoystick.povUp().whileTrue(arm.sysidQuasistatic(Direction.kForward));
        coDriverJoystick.povDown().whileTrue(arm.sysidQuasistatic(Direction.kReverse));
        coDriverJoystick.povLeft().whileTrue(arm.sysidDynamic(Direction.kReverse));
        coDriverJoystick.povRight().whileTrue(arm.sysidDynamic(Direction.kForward));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    
        driverJoystick.leftBumper().onTrue(Commands.runOnce(SignalLogger::start));

        driverJoystick.rightBumper().onTrue(Commands.runOnce(SignalLogger::stop));
    
    
    }

    public Command getAutonomousCommand() {
        return autoSelector.selectedCommand();
    }
}
