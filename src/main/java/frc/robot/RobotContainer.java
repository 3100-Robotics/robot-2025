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

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.superstructureConstants.States;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.ProtectionArms;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private final double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private final double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.09).withRotationalDeadband(MaxAngularRate * 0.09) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Climber climber = new Climber();
    public final Algae algae = new Algae();

    public final ProtectionArms protectionArms = new ProtectionArms();

    public final Elevator elevator = new Elevator();
    public final Arm arm = new Arm();
    public final Superstructure superstructure = new Superstructure(elevator, arm);

    public final Vision downCamera = new Vision("Down", new Transform3d(new Translation3d(
        Units.inchesToMeters(-9.25),
        Units.inchesToMeters(-10.401),
        Units.inchesToMeters(11.25)),
        new Rotation3d(0, Math.toRadians(0), Math.toRadians(-90))));

    public final Vision gamePieceCamera = new Vision("up", new Transform3d());

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
                algae.set(0),
                superstructure.goToPos(States.resting, "neither"));
    }
    
    public Command scoreAlgae(States scoringPos, String side) {
        return Commands.sequence( 
            protectionArms.set(side),
            superstructure.goToPos(scoringPos, side),
            Commands.waitSeconds(0.05),
            algae.set(0.68),
            Commands.waitSeconds(0.4),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"),
            protectionArms.restArm());
    }

    public Command scoreAlgaeAuto(String side) {
        return Commands.sequence( 
            protectionArms.set(side),
            superstructure.goToPos(States.algaeToBardgeAuto, side),
            Commands.waitSeconds(0.05),
            algae.set(1),
            Commands.waitSeconds(0.4),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"),
            protectionArms.restArm());
    }

    public Command scoreAlgaeProcessor(String side) {
        return Commands.sequence(
            drivetrain.applyRequest(() -> driveRobotCentric.withVelocityY(side.equals("left") ? -1.5 : 1.5))
                .withTimeout(0.3).asProxy(),
            drivetrain.applyRequest(() -> driveRobotCentric.withVelocityY(0))
                .withTimeout(0.001).asProxy(),
            superstructure.goToPos(States.algaeToProcessor, side),
            drivetrain.applyRequest(() -> driveRobotCentric.withVelocityY(side.equals("left") ? 1.5 : -1.5))
                .withTimeout(0.1).asProxy(),
            drivetrain.applyRequest(() -> driveRobotCentric.withVelocityY(0))
                .withTimeout(0.001).asProxy(),  
            algae.set(-0.25),
            Commands.waitSeconds(0.1),
            algae.set(0.5),
            Commands.waitSeconds(0.5),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"));
    }

    public Command collectCoral(States collectingState) {
        return Commands.sequence(
            algae.set(1),
            superstructure.goToPos(collectingState, "right"),
            Commands.waitUntil(algae.currentHit()),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"));
    }

    public Command scoreCoral() {
        return Commands.sequence(
            drivetrain.applyRequest(() -> driveRobotCentric.withVelocityY(0.75))
                .withTimeout(0.2).asProxy(),
            drivetrain.applyRequest(() -> driveRobotCentric.withVelocityY(0))
                .withTimeout(0.001).asProxy(),
            superstructure.goToPos(States.coralToL1, "right"),
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
            collectAlgae(States.algaeFromReefHigh, "right")));

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

        startToAlgae1.done().onTrue(Commands.parallel(
            collectAlgae(States.algaeFromReefLow, "right"),
            Commands.sequence(
                Commands.waitUntil(algae.limitHit()),
                algae1ToScore.resetOdometry(),
                algae1ToScore.spawnCmd())));

        algae1ToScore.done().onTrue(Commands.sequence(
                Commands.waitUntil(() -> superstructure.getState().equals(States.resting)),
                Commands.waitSeconds(0.5),
                // Commands.waitUntil(superstructure.atSetpoint()),
                // scoreAlgae(States.algaeToBardge, "left"),
                scoreToAlgae2.spawnCmd()));

        scoreToAlgae2.done().onTrue(Commands.parallel(
            collectAlgae(States.algaeFromReefHigh, "right"),
            Commands.sequence(
                Commands.waitUntil(algae.limitHit()),
                algae2ToScore.resetOdometry(),
                algae2ToScore.spawnCmd())));

        algae2ToScore.done().onTrue(Commands.sequence(
            Commands.waitUntil(() -> superstructure.getState().equals(States.resting)),
            Commands.waitSeconds(0.5),
            // Commands.waitUntil(superstructure.atSetpoint()),
            // scoreAlgae(States.algaeToBardge, "left"),
            scoreToAlgae3.spawnCmd()));

        scoreToAlgae3.done().onTrue(Commands.sequence(
            collectAlgae(States.algaeFromReefHigh, "right")
        ));

        return routine;
    }

    public AutoRoutine score3Algae() {
        AutoRoutine routine = autoFactory.newRoutine("2 algae");
        AutoTrajectory startToAlgae1 = routine.trajectory("start-algae1");
        AutoTrajectory algae1ToScore = routine.trajectory("algae1-score");
        AutoTrajectory scoreToAlgae2 = routine.trajectory("score-algae2");
        AutoTrajectory algae2ToScore = routine.trajectory("algae2-score");
        AutoTrajectory scoreToAlgae3 = routine.trajectory("score-algae3");
        AutoTrajectory algae3ToScore = routine.trajectory("algae3-score");

        startToAlgae1.atTimeBeforeEnd(0.25).onTrue(Commands.sequence(
            algae.set(-1),
            superstructure.goToPos(States.algaeFromReefLow, "right").until(algae.limitHit()),
            Commands.waitUntil(algae.limitHit()),
            algae.set(0)));

        scoreToAlgae3.atTimeBeforeEnd(0.6).onTrue(Commands.sequence(
            algae.set(-1),
            superstructure.goToPos(States.algaeFromReefHigh, "right").until(algae.limitHit()),
            Commands.waitUntil(algae.limitHit()),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither")));

        algae1ToScore.atTimeBeforeEnd(0.75).onTrue(scoreAlgaeAuto("left"));
        algae2ToScore.atTimeBeforeEnd(0.75).onTrue(scoreAlgaeAuto("left"));
        algae3ToScore.atTimeBeforeEnd(0.75).onTrue(scoreAlgaeAuto("left"));

        routine.active().onTrue(Commands.sequence(
                startToAlgae1.resetOdometry(),
                startToAlgae1.cmd()));

        startToAlgae1.done().onTrue(Commands.parallel(
            Commands.sequence(
                Commands.waitUntil(algae.limitHit()),
                algae1ToScore.resetOdometry(),
                algae1ToScore.spawnCmd())));

        algae1ToScore.done().onTrue(Commands.sequence(
                Commands.waitUntil(() -> superstructure.getState().equals(States.resting)),
                Commands.waitSeconds(0.5),
                scoreToAlgae2.spawnCmd()));

        scoreToAlgae2.done().onTrue(Commands.parallel(
            Commands.sequence(
                algae.set(-1),
                superstructure.goToPos(States.algaeFromReefHigh, "right").until(algae.limitHit()),
                Commands.waitUntil(algae.limitHit()),
                algae.set(0),
                superstructure.goToPos(States.resting, "neither")),
            Commands.sequence(
                Commands.waitUntil(algae.limitHit()),
                algae2ToScore.resetOdometry(),
                algae2ToScore.spawnCmd())));

        algae2ToScore.done().onTrue(Commands.sequence(
            Commands.waitUntil(() -> superstructure.getState().equals(States.resting)),
            Commands.waitSeconds(0.5),
            scoreToAlgae3.spawnCmd()));

        scoreToAlgae3.done().onTrue(Commands.parallel(
            Commands.sequence(
                Commands.waitUntil(algae.limitHit()),
                algae3ToScore.resetOdometry(),
                algae3ToScore.spawnCmd())));

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

    public AutoRoutine scoreManyCoral() {
        AutoRoutine routine = autoFactory.newRoutine("many coral");

        AutoTrajectory startToScore1 = routine.trajectory("start2-score1");
        AutoTrajectory score1ToCollect = routine.trajectory("score1-collect");
        AutoTrajectory collectToscore2 = routine.trajectory("collect-score2");
        AutoTrajectory score2ToCollect = routine.trajectory("score2-collect");

        routine.active().onTrue(Commands.sequence(
            startToScore1.resetOdometry(),
            startToScore1.spawnCmd()));
        
        startToScore1.done().onTrue(Commands.sequence(
            scoreCoral(),
            score1ToCollect.spawnCmd()
                .alongWith(collectCoral(States.coralFromHp))));
        
        routine.observe(algae.currentHit()).onTrue(
            collectToscore2.spawnCmd());

        collectToscore2.done().onTrue(Commands.sequence(
            scoreCoral(),
            score2ToCollect.spawnCmd()
                .alongWith(collectCoral(States.coralToL1))));

        return routine;
    }

    private void configureAutonomous() {
        SmartDashboard.putData("auto selector", autoSelector);

        autoSelector.addCmd("nothing", Commands::none);
        autoSelector.addRoutine("leave", this::leave);
        autoSelector.addRoutine("score 1 algae", this::score1Algae);
        autoSelector.addRoutine("score 1.5 algae", this::score15Algae);
        autoSelector.addRoutine("score 2 algae", this::score2Algae);
        autoSelector.addRoutine("score 3 algae", this::score3Algae);
        autoSelector.addRoutine("score algae 1, safe", this::scoreAlgae1Leave);
        autoSelector.addRoutine("score many coral", this::scoreManyCoral);

        autoSelector.select("score 2 algae");
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

        coDriverJoystick.povRight().onTrue(Commands.sequence(
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"),
            superstructure.goToPos(States.rezeroElevator, "neither").until(elevator.atBottom()),
            superstructure.goToPos(States.resting, "neigher")));

        ///////////
        // ALGAE //
        ///////////

        driverJoystick.povUp().onTrue(Commands.sequence(
            algae.set(-1),
            Commands.waitUntil(algae.limitHit()),
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
        coDriverJoystick.rightBumper().and(driverJoystick.leftBumper()).onTrue(scoreAlgaeProcessor("left"));
        // right
        coDriverJoystick.rightBumper().and(driverJoystick.rightBumper()).onTrue(scoreAlgaeProcessor("right"));

        // barge
        // left
        coDriverJoystick.leftBumper().and(driverJoystick.leftBumper()).onTrue(scoreAlgae(States.algaeToBardge, "left"));
        // right
        coDriverJoystick.leftBumper().and(driverJoystick.rightBumper()).onTrue(scoreAlgae(States.algaeToBardge, "right"));

        coDriverJoystick.b().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromLollipop, "left"));

        coDriverJoystick.b().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromLollipop, "right"));

        ///////////
        // CORAL //
        ///////////

        coDriverJoystick.povDown().onTrue(collectCoral(States.coralFromGround));

        coDriverJoystick.povLeft().onTrue(collectCoral(States.coralFromHp));

        coDriverJoystick.povUp().onTrue(scoreCoral());

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
