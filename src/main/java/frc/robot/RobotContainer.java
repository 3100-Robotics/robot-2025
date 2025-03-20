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

import au.grapplerobotics.CanBridge;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.States;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.AppendageFoam;
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

    public final AppendageFoam appendage_foam = new AppendageFoam();

    public final Elevator elevator = new Elevator();
    public final Arm arm = new Arm();
    public final Superstructure superstructure = new Superstructure(elevator, arm);

    public final Vision downCamera = new Vision("Down", new Transform3d(new Translation3d(
        Units.inchesToMeters(-9.25),
        Units.inchesToMeters(-10.401),
        Units.inchesToMeters(11.25)),
        new Rotation3d(0, Math.toRadians(0), Math.toRadians(-90))));

    public final Vision gamePieceCamera = new Vision("game piece", new Transform3d());

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
                superstructure.goToPos(collectingState, side).until(algae.limitHit()),
                Commands.waitUntil(algae.limitHit()),
                // Commands.waitSeconds(0.15),
                algae.set(0),
                superstructure.goToPos(States.resting, "neither"));
    }

    public Command scoreAlgae(States scoringPos, String side) {
        return Commands.sequence(
            appendage_foam.setAngle(0),
            superstructure.goToPos(scoringPos, side),
            algae.set(-0.5),
            Commands.waitSeconds(0.1),
            algae.set(0.5),
            Commands.waitSeconds(0.5),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"),
            appendage_foam.setAngle(90));
    }

    public Command collectAlgaeReefHigh(String side) {
        return Commands.sequence(
                algae.set(-1),
                superstructure.goToPos(States.algaeFromReefHighStep1, side),
                Commands.print("first state done"),
                superstructure.goToPos(States.algaeFromReefHighStep2, side).until(algae.limitHit()),
                Commands.waitUntil(algae.limitHit()),
                // Commands.waitSeconds(0.15),
                algae.set(0),
                superstructure.goToPos(States.resting, "neither"));
    }

    public Command collectCoral(States collectingState) {
        return Commands.sequence(
            algae.set(0.75),
            superstructure.goToPos(collectingState, "right"),
            Commands.waitUntil(algae.currentHit()),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"));
    }

    public Command scoreCoral(States scoringPos) {
        return Commands.sequence(
            superstructure.goToPos(scoringPos, "right"),
            algae.set(-0.25),
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
                collectAlgae(States.algaeFromReefLow, "right"),
                algae1ToScore.resetOdometry(),
                algae1ToScore.spawnCmd()));

        algae1ToScore.done().onTrue(Commands.sequence(
                scoreAlgae(States.algaeToBardge, "left")));

        return routine;
    }

    public AutoRoutine score15Algae() {
        AutoRoutine routine = autoFactory.newRoutine("2 algae");
        AutoTrajectory startToAlgae1 = routine.trajectory("start-algae1");
        AutoTrajectory algae1ToScore = routine.trajectory("algae1-score");
        AutoTrajectory scoreToAlgae2 = routine.trajectory("score-algae2");
        AutoTrajectory algae2ToScore = routine.trajectory("algae2-score");

        routine.active().onTrue(Commands.sequence(
                startToAlgae1.resetOdometry(),
                startToAlgae1.cmd()));

        startToAlgae1.done().onTrue(Commands.sequence(
                collectAlgae(States.algaeFromReefLow, "right"),
                algae1ToScore.resetOdometry(),
                algae1ToScore.spawnCmd()));

        algae1ToScore.done().onTrue(Commands.sequence(
                scoreAlgae(States.algaeToBardge, "left"),
                scoreToAlgae2.cmd()));

        scoreToAlgae2.done().onTrue(Commands.sequence(
            Commands.print("algae 1 done"),
            collectAlgaeReefHigh("right")));

        // algae1.active().onFalse(Commands.sequence(
        //         scoreToAlgae2.spawnCmd()));

        // algae2ToScore.done().onTrue(Commands.sequence(
        
        //         algae2ToScore.spawnCmd()));

        // algae2ToScore.done().onTrue(scoreAlgae(States.algaeToBardge, "right"));

        return routine;
    }

    public AutoRoutine score2Algae() {
        AutoRoutine routine = autoFactory.newRoutine("2 algae");
        AutoTrajectory startToAlgae1 = routine.trajectory("start-algae1");
        AutoTrajectory algae1ToScore = routine.trajectory("algae1-score");
        AutoTrajectory scoreToAlgae2 = routine.trajectory("score-algae2");
        AutoTrajectory algae2ToScore = routine.trajectory("algae2-score");
        AutoTrajectory scoreToAlgae3 = routine.trajectory("score-algae3");

        algae1ToScore.atTimeBeforeEnd(1).onTrue(scoreAlgae(States.algaeToBardge, "left"));
        algae2ToScore.atTimeBeforeEnd(1).onTrue(scoreAlgae(States.algaeToBardge, "left"));

        routine.active().onTrue(Commands.sequence(
                startToAlgae1.resetOdometry(),
                startToAlgae1.cmd()));

        startToAlgae1.done().onTrue(Commands.sequence(
                collectAlgae(States.algaeFromReefLow, "right"),
                algae1ToScore.resetOdometry(),
                algae1ToScore.spawnCmd()));

        algae1ToScore.done().onTrue(Commands.sequence(
                Commands.waitUntil(() -> superstructure.getState().equals(States.resting)),
                Commands.waitSeconds(0.5),
                // Commands.waitUntil(superstructure.atSetpoint()),
                // scoreAlgae(States.algaeToBardge, "left"),
                scoreToAlgae2.spawnCmd()));

        scoreToAlgae2.done().onTrue(Commands.sequence(
            collectAlgaeReefHigh("right"),
            algae2ToScore.resetOdometry(),
            algae2ToScore.spawnCmd()));

        algae2ToScore.done().onTrue(Commands.sequence(
            Commands.waitUntil(() -> superstructure.getState().equals(States.resting)),
            Commands.waitSeconds(0.5),
            // Commands.waitUntil(superstructure.atSetpoint()),
            // scoreAlgae(States.algaeToBardge, "left"),
            scoreToAlgae3.spawnCmd()));

        scoreToAlgae3.done().onTrue(Commands.sequence(
            collectAlgaeReefHigh("right")
        ));

        return routine;
    }

    public AutoRoutine scoreAlgae1Leave() {
        AutoRoutine routine = autoFactory.newRoutine("1 algae");

        AutoTrajectory startToAlgae1 = routine.trajectory("start-algae1");
        AutoTrajectory algae1ToScore = routine.trajectory("algae1-score");
        AutoTrajectory scoreToSafe = routine.trajectory("score-safe");

        routine.active().onTrue(Commands.sequence(
                startToAlgae1.resetOdometry(),
                startToAlgae1.cmd()));

        startToAlgae1.done().onTrue(Commands.sequence(
                collectAlgae(States.algaeFromReefLow, "right"),
                algae1ToScore.resetOdometry(),
                algae1ToScore.spawnCmd()));

        algae1ToScore.done().onTrue(Commands.sequence(
                scoreAlgae(States.algaeToBardge, "left"),
                scoreToSafe.cmd()));

        return routine;
    }

    private void configureAutonomous() {
        SmartDashboard.putData("auto selector", autoSelector);

        autoSelector.addCmd("nothing", Commands::none);
        autoSelector.addRoutine("leave", this::leave);
        autoSelector.addRoutine("score algae 1", this::score1Algae);
        autoSelector.addRoutine("score algae 1.5", this::score15Algae);
        autoSelector.addRoutine("score 2 algae", this::score2Algae);

        autoSelector.select("score algae 1,2");
        autoSelector.addRoutine("score algae 1, safe", this::scoreAlgae1Leave);
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

        climber.setDefaultCommand(climber.setSpeed(coDriverJoystick::getLeftX));

        driverJoystick.a().onTrue(superstructure.goToPos(States.algaeFromLollipop, "right"));
        driverJoystick.b().onTrue(Commands.none());

        driverJoystick.y().whileTrue(drivetrain.allignToBarge());

        driverJoystick.x().and(driverJoystick.leftBumper()).whileTrue(drivetrain.driveToGamePiece(gamePieceCamera::getLatestResult, algae.limitHit(), "left"));
        driverJoystick.x().and(driverJoystick.rightBumper()).whileTrue(drivetrain.driveToGamePiece(gamePieceCamera::getLatestResult, algae.limitHit(), "right"));

        // driverJoystick.a().onTrue(climber.goToPos(0)); unused at the moment
        // driverJoystick.b().onTrue(climber.goToPos(3));

        ///////////
        // ALGAE //
        ///////////

        driverJoystick.povUp().onTrue(Commands.sequence(
            algae.set(-1),
            Commands.waitUntil(algae.limitHit()),
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
        coDriverJoystick.y().and(driverJoystick.leftBumper()).onTrue(collectAlgaeReefHigh("left"));
        // right
        coDriverJoystick.y().and(driverJoystick.rightBumper()).onTrue(collectAlgaeReefHigh("right"));

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
            superstructure.goToPos(States.algaeFromLollipop, "left").until(algae.limitHit()),
            Commands.waitUntil(algae.limitHit()),
            Commands.waitSeconds(0.15),
            superstructure.goToPos(States.resting, "neither"),
            algae.set(0)));

        coDriverJoystick.b().and(driverJoystick.rightBumper()).onTrue(Commands.sequence(
            algae.set(-1),
            superstructure.goToPos(States.algaeFromLollipop, "right").until(algae.limitHit()),
            Commands.waitUntil(algae.limitHit()),
            Commands.waitSeconds(0.15),
            superstructure.goToPos(States.resting, "neither"),
            algae.set(0)));

        ///////////
        // CORAL //
        ///////////

        coDriverJoystick.povDown().onTrue(collectCoral(States.coralFromGround));

        coDriverJoystick.povLeft().onTrue(collectCoral(States.coralFromHp));

        coDriverJoystick.povUp().onTrue(scoreCoral(States.coralToL1));

        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

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
