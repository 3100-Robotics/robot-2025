// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import java.util.function.Supplier;

import choreo.auto.AutoChooser;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.Constants.superstructureConstants.States;
import frc.robot.generated.TunerConstants;
import frc.robot.math.LocatorEngine;
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
            .withDeadband(MaxSpeed * 0.09).withRotationalDeadband(MaxAngularRate * 0.09) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.RobotCentric driveRobotCentric = new SwerveRequest.RobotCentric();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final Climber climber = new Climber();
    public final Algae algae = new Algae();

    public final Elevator elevator = new Elevator();
    public final Arm arm = new Arm();
    public final Superstructure superstructure = new Superstructure(elevator, arm);

    public final Vision downCamera = new Vision("Down", new Transform3d(new Translation3d(
        Units.inchesToMeters(-9.25),
        Units.inchesToMeters(-10.401),
        Units.inchesToMeters(14.25)),
        new Rotation3d(0, Math.toRadians(20), Math.toRadians(-90))));

        public final Vision gamePieceCamera = new Vision("up", new Transform3d());

    private final CommandXboxController driverJoystick = new CommandXboxController(0);
    private final CommandXboxController coDriverJoystick = new CommandXboxController(1);

    public final AutoChooser autoSlector = new AutoChooser();
    
    private AutoRoutines autoroutines;

    public LocatorEngine locengine = new LocatorEngine(()->drivetrain.getPos());

    public RobotContainer(Boolean isreal) {
        autoroutines = new AutoRoutines(this);
        configureBindings();
        configureAutonomous();
    }

    public String algaeSide() {
        String side = locengine.bargeSideLeft() ? "left" : "left";
        // DataLogManager.log("Pushing algae " + side);
        return side;
    }


    public Command collectAlgae(States collectingState, Supplier<String> side) {
        return Commands.sequence(
                algae.set(-1),
                superstructure.goToPos(collectingState, side.get()).until(algae.limitHit()),
                Commands.waitUntil(algae.limitHit()),
                algae.set(0),
                superstructure.goToPos(States.resting, "neither"));
    }
    
    public Command scoreAlgae(States scoringPos, Supplier<String> side) {
        return Commands.sequence(
            superstructure.goToPos(scoringPos, side.get()),
            Commands.waitSeconds(0.05),
            algae.set(0.68),
            Commands.waitSeconds(0.4),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"));
    }

    public Command scoreAlgaeAuto(Supplier<String> side) {
        return Commands.sequence(
            superstructure.goToPos(States.algaeToBardgeAuto, side.get()),
            Commands.waitSeconds(0.05),
            algae.set(0.9),
            Commands.waitSeconds(0.4),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"));
    }

    public Command scoreAlgaeProcessor(Supplier<String> side) {
        return Commands.sequence(
            drivetrain.applyRequest(() -> driveRobotCentric.withVelocityY(side.get().equals("left") ? -1.5 : 1.5))
                .withTimeout(0.3).asProxy(),
            drivetrain.applyRequest(() -> driveRobotCentric.withVelocityY(0))
                .withTimeout(0.001).asProxy(),
            superstructure.goToPos(States.algaeToProcessor, side.get()),
            drivetrain.applyRequest(() -> driveRobotCentric.withVelocityY(side.get().equals("left") ? 1.5 : -1.5))
                .withTimeout(0.1).asProxy(),
            drivetrain.applyRequest(() -> driveRobotCentric.withVelocityY(0))
                .withTimeout(0.001).asProxy(),
            algae.set(0.5),
            Commands.waitSeconds(0.4),
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
            Commands.waitSeconds(0.3),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"));
    }

    public Command scoreCoralAuto() {
        return Commands.sequence(
            superstructure.goToPos(States.coralToL1, "right"),
            algae.set(-0.25),
            Commands.waitSeconds(0.3),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"));
    }


    private void configureAutonomous() {
        SmartDashboard.putData("auto selector", autoSlector);

        autoSlector.addCmd("nothing", Commands::none);
        autoSlector.addRoutine("leave", autoroutines::leave);
        autoSlector.addRoutine("score 1 algae", autoroutines::scoreAlgae1Leave);
        autoSlector.addRoutine("score 1.5 algae", autoroutines::score15Algae);
        autoSlector.addRoutine("score 2 algae", autoroutines::score2Algae);
        autoSlector.addRoutine("score 3 algae", autoroutines::score3Algae);
        autoSlector.addRoutine("score many coral", autoroutines::scoreManyCoral);

        autoSlector.select("score 3 algae");
    }

    private void configureBindings() {
        SmartDashboard.putString("binding", "normal");
        SmartDashboard.getString("binding", "normal");
        switch (SmartDashboard.getString("binding", "normal")) {
            case "normal":
                configureNormalBindings();
                break;
            case "sysid":
                configureSysidBindings();
                break;
            case "even":
                configureEvenBindings();
                break;
            default:
                configureNormalBindings();
        }
    }

    private void configureEvenBindings() {
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

        // climber 
        driverJoystick.a().onTrue(Commands.parallel(
            superstructure.goToPos(States.algaeFromLollipop, "right"),
            Commands.sequence(
                Commands.waitSeconds(0.25),
                climber.goToPos(8.1148).until(climber.atSetpoint()))));
        
        driverJoystick.b().onTrue(climber.goToPos(0).until(coDriverJoystick.leftStick()));

        // barge alignment (not great)
        driverJoystick.y().whileTrue(drivetrain.allignToBarge());

        // auto collect
        driverJoystick.x().and(driverJoystick.leftBumper()).whileTrue(drivetrain.driveToGamePiece(gamePieceCamera::getLatestResult, algae.limitHit(), "left"));
        driverJoystick.x().and(driverJoystick.rightBumper()).whileTrue(drivetrain.driveToGamePiece(gamePieceCamera::getLatestResult, algae.limitHit(), "right"));

        // idle
        coDriverJoystick.povRight().onTrue(Commands.sequence(
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"),
            superstructure.goToPos(States.rezeroElevator, "neither").until(elevator.atBottom()),
            superstructure.goToPos(States.resting, "neigher")));

        
        // driverJoystick.povLeft().onTrue(protectionArms.set("left"));
        // driverJoystick.povRight().onTrue(protectionArms.set("right"));
        // driverJoystick.povDown().onTrue(protectionArms.restArm());

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
        coDriverJoystick.a().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromGround, ()->"left"));
        // right
        coDriverJoystick.a().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromGround, ()->"right"));

        // between l2 and l3
        // left
        coDriverJoystick.x().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromReefLow, ()->"left"));
        // right
        coDriverJoystick.x().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromReefLow, ()->"right"));

        // between l3 and l4
        // left
        coDriverJoystick.y().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromReefHigh, ()->"left"));
        // right
        coDriverJoystick.y().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromReefHigh, ()->"right"));

        // scoring

        // processor
        // left
        coDriverJoystick.rightBumper().and(driverJoystick.leftBumper()).onTrue(scoreAlgaeProcessor(()->"left"));
        // right
        coDriverJoystick.rightBumper().and(driverJoystick.rightBumper()).onTrue(scoreAlgaeProcessor(()->"right"));

        // barge
        // left / left
        coDriverJoystick.leftBumper().onTrue(scoreAlgae(States.algaeToBardge, ()->algaeSide()));

        // lollipop
        // left
        coDriverJoystick.b().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromLollipop, ()->"left"));
        // right
        coDriverJoystick.b().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromLollipop, ()->"right"));

        ///////////
        // CORAL //
        ///////////

        coDriverJoystick.povDown().onTrue(collectCoral(States.coralFromGround));

        coDriverJoystick.povLeft().onTrue(collectCoral(States.coralFromHp));

        coDriverJoystick.povUp().onTrue(scoreCoral());

        drivetrain.registerTelemetry(logger::telemeterize);
    }

    private void configureNormalBindings() {
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

        // climber 
        driverJoystick.a().onTrue(Commands.parallel(
            superstructure.goToPos(States.algaeFromLollipop, "right"),
            Commands.sequence(
                Commands.waitSeconds(0.25),
                climber.goToPos(8.1148).until(climber.atSetpoint()))));
        
        driverJoystick.b().onTrue(climber.goToPos(0).until(coDriverJoystick.leftStick()));

        // barge alignment (not great)
        driverJoystick.y().whileTrue(drivetrain.allignToBarge());

        // auto collect
        driverJoystick.x().and(driverJoystick.leftBumper()).whileTrue(drivetrain.driveToGamePiece(gamePieceCamera::getLatestResult, algae.limitHit(), "left"));
        driverJoystick.x().and(driverJoystick.rightBumper()).whileTrue(drivetrain.driveToGamePiece(gamePieceCamera::getLatestResult, algae.limitHit(), "right"));

        // idle
        coDriverJoystick.povRight().onTrue(Commands.sequence(
            algae.set(0),
            superstructure.goToPos(States.resting, "neither"),
            superstructure.goToPos(States.rezeroElevator, "neither").until(elevator.atBottom()),
            superstructure.goToPos(States.resting, "neigher")));

        
        // driverJoystick.povLeft().onTrue(protectionArms.set("left"));
        // driverJoystick.povRight().onTrue(protectionArms.set("right"));
        // driverJoystick.povDown().onTrue(protectionArms.restArm());

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
        coDriverJoystick.a().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromGround, ()->"left"));
        // right
        coDriverJoystick.a().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromGround, ()->"right"));

        // between l2 and l3
        // left
        coDriverJoystick.x().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromReefLow, ()->"left"));
        // right
        coDriverJoystick.x().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromReefLow, ()->"right"));

        // between l3 and l4
        // left
        coDriverJoystick.y().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromReefHigh, ()->"left"));
        // right
        coDriverJoystick.y().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromReefHigh, ()->"right"));

        // scoring

        // processor
        // left
        coDriverJoystick.rightBumper().and(driverJoystick.leftBumper()).onTrue(scoreAlgaeProcessor(()->"left"));
        // right
        coDriverJoystick.rightBumper().and(driverJoystick.rightBumper()).onTrue(scoreAlgaeProcessor(()->"right"));

        // barge
        // left
        coDriverJoystick.leftBumper().and(driverJoystick.leftBumper()).onTrue(scoreAlgae(States.algaeToBardge, ()->"left"));
        // right
        coDriverJoystick.leftBumper().and(driverJoystick.rightBumper()).onTrue(scoreAlgae(States.algaeToBardge, ()->"right"));

        // lollipop
        // left
        coDriverJoystick.b().and(driverJoystick.leftBumper()).onTrue(collectAlgae(States.algaeFromLollipop, ()->"left"));
        // right
        coDriverJoystick.b().and(driverJoystick.rightBumper()).onTrue(collectAlgae(States.algaeFromLollipop, ()->"right"));

        ///////////
        // CORAL //
        ///////////

        coDriverJoystick.povDown().onTrue(collectCoral(States.coralFromGround));

        coDriverJoystick.povLeft().onTrue(collectCoral(States.coralFromHp));

        coDriverJoystick.povUp().onTrue(scoreCoral());

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
        return autoSlector.selectedCommand();
    }
}
