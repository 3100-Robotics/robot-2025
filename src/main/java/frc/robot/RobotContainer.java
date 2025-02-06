// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import choreo.auto.AutoChooser;
import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
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
import frc.robot.subsystems.Coral;
import frc.robot.subsystems.Elevator;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

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
    public final Coral coral = new Coral();

    public final Elevator elevator = new Elevator();
    public final Arm arm = new Arm();
    public final Superstructure superstructure = new Superstructure(elevator, arm);

    public final Vision vision = new Vision(drivetrain::getPos, drivetrain.getField());

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController coDriverJoystick = new CommandXboxController(1);

    private final AutoChooser autoSelector = new AutoChooser();

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
        configureAutonomous();
    }

    public Command scoreCoral(States scoringState) {
        return Commands.sequence(
                superstructure.goToPos(scoringState, "neither"),
                coral.set(0.5),
                Commands.waitSeconds(0.25),
                superstructure.goToPos(States.resting, "neither"));
    }

    public Command collectAlgae(States collectingState, String side) {
        return Commands.sequence(
                algae.set(-0.5),
                superstructure.goToPos(collectingState, side).until(algae.currentHit()),
                Commands.waitUntil(algae.currentHit()),
                algae.set(0),
                superstructure.goToPos(States.resting, "neither"));
    }

    public Command scoreAlgae(States scoringPos, String side) {
        return Commands.sequence(
                superstructure.goToPos(scoringPos, side),
                algae.set(0.5),
                Commands.waitSeconds(0.25),
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

    public AutoRoutine scorePreload() {
        AutoRoutine routine = autoFactory.newRoutine("scorePreload");

        AutoTrajectory startToCoral1 = routine.trajectory("start->coral1");
        AutoTrajectory coral1ToEnd = routine.trajectory("coral1->end");

        routine.active().onTrue(
                Commands.sequence(
                        startToCoral1.resetOdometry(),
                        startToCoral1.cmd()));

        startToCoral1.done().onTrue(Commands.sequence(
                scoreCoral(States.coralReefL4),
                coral1ToEnd.cmd()));

        return routine;
    }

    private void configureAutonomous() {
        SmartDashboard.putData(autoSelector);

        autoSelector.addCmd("nothing", Commands::none);
        autoSelector.addRoutine("leave", this::leave);
        autoSelector.addRoutine("score preload", this::scorePreload);
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

        ///////////
        // CORAL //
        ///////////

        // collection
        coDriverJoystick.povUp().onTrue(Commands.sequence(
                superstructure.goToPos(States.coralHumanPlayer, "neither"),
                coral.set(-0.5),
                Commands.waitUntil(coral.currentHit()),
                superstructure.goToPos(States.resting, "neither")));

        // scoring
        // l2
        coDriverJoystick.povDown().onTrue(scoreCoral(States.coralReefL2));

        // l3
        coDriverJoystick.povLeft().onTrue(scoreCoral(States.coralReefL3));

        // l4
        coDriverJoystick.povRight().onTrue(scoreCoral(States.coralReefL4));

        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoystick.back().and(driverJoystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoystick.back().and(driverJoystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoystick.start().and(driverJoystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoystick.start().and(driverJoystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
    }

    public Command getAutonomousCommand() {
        return autoSelector.selectedCommand();
    }
}
