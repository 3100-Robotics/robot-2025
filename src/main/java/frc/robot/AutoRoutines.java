package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.superstructureConstants.States;
import frc.robot.subsystems.Algae;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Drivetrain;
import frc.robot.subsystems.Elevator;

public class AutoRoutines {
    private final RobotContainer robotcontainer;
    private AutoFactory autoFactory;

    public final Drivetrain drivetrain;
    public final Climber climber;
    public final Algae algae;
    public final Elevator elevator;
    public final Arm arm;
    public final Superstructure superstructure;

    public AutoRoutines(RobotContainer _robotcontainer) {
        robotcontainer = _robotcontainer;

        drivetrain = robotcontainer.drivetrain;
        climber = robotcontainer.climber;
        algae = robotcontainer.algae;
        elevator = robotcontainer.elevator;
        arm = robotcontainer.arm;
        superstructure = robotcontainer.superstructure;

        autoFactory = new AutoFactory(
            drivetrain::getPos, // A function that returns the current robot pose
            drivetrain::resetPose, // A function that resets the current robot pose to the provided Pose2d
            drivetrain::followTrajectory, // The drive subsystem trajectory follower
            true, // If alliance flipping should be enabled
            drivetrain // The drive subsystem
        );
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
                robotcontainer.collectAlgae(States.algaeFromReefLow, ()->"right"),
                algae1ToScore.resetOdometry(),
                algae1ToScore.spawnCmd()));

        algae1ToScore.done().onTrue(Commands.sequence(
                robotcontainer.scoreAlgae(States.algaeToBardge, ()->"left")));

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
                robotcontainer.collectAlgae(States.algaeFromReefLow, ()->"right"),
                algae1ToScore.resetOdometry(),
                algae1ToScore.spawnCmd()));

        algae1ToScore.done().onTrue(Commands.sequence(
                robotcontainer.scoreAlgae(States.algaeToBardge, ()->"left"),
                scoreToAlgae2.cmd()));

        scoreToAlgae2.done().onTrue(Commands.sequence(
            Commands.print("algae 1 done"),
            robotcontainer.collectAlgae(States.algaeFromReefHigh, ()->"right")));

        return routine;
    }

    public AutoRoutine score2Algae() {
        AutoRoutine routine = autoFactory.newRoutine("2 algae");
        AutoTrajectory startToAlgae1 = routine.trajectory("start-algae1");
        AutoTrajectory algae1ToScore = routine.trajectory("algae1-score");
        AutoTrajectory scoreToAlgae2 = routine.trajectory("score-algae2");
        AutoTrajectory algae2ToScore = routine.trajectory("algae2-score");
        AutoTrajectory scoreToAlgae3 = routine.trajectory("score-algae3");

        algae1ToScore.atTimeBeforeEnd(1).onTrue(robotcontainer.scoreAlgae(States.algaeToBardge, ()->"left"));
        algae2ToScore.atTimeBeforeEnd(1).onTrue(robotcontainer.scoreAlgae(States.algaeToBardge, ()->"left"));

        routine.active().onTrue(Commands.sequence(
                startToAlgae1.resetOdometry(),
                startToAlgae1.cmd()));

        startToAlgae1.done().onTrue(Commands.parallel(
            robotcontainer.collectAlgae(States.algaeFromReefLow, ()->"right"),
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
            robotcontainer.collectAlgae(States.algaeFromReefHigh, ()->"right"),
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
            robotcontainer.collectAlgae(States.algaeFromReefHigh, ()->"right")
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

        startToAlgae1.atTimeBeforeEnd(0.5).onTrue(Commands.sequence(
            algae.set(-1),
            superstructure.goToPos(States.algaeFromReefLow, "right").until(algae.limitHit()),
            Commands.waitUntil(algae.limitHit()),
            algae.set(0)));

        scoreToAlgae2.atTimeBeforeEnd(0.5).onTrue(Commands.sequence(
            algae.set(-1),
            superstructure.goToPos(States.algaeFromReefHigh, "right").until(algae.limitHit()),
            Commands.waitUntil(algae.limitHit()),
            algae.set(0)));
            // superstructure.goToPos(States.resting, "neither")));
        
        scoreToAlgae3.atTimeBeforeEnd(0.8).onTrue(Commands.sequence(
            algae.set(-1),
            superstructure.goToPos(States.algaeFromReefHigh, "right").until(algae.limitHit()),
            Commands.waitUntil(algae.limitHit()),
            algae.set(0),
            superstructure.goToPos(States.resting, "neither")));

        algae1ToScore.atTimeBeforeEnd(0.8).onTrue(robotcontainer.scoreAlgaeAuto(()->"left"));
        algae2ToScore.atTimeBeforeEnd(0.4).onTrue(robotcontainer.scoreAlgaeAuto(()->"left"));
        algae3ToScore.atTimeBeforeEnd(0.75).onTrue(robotcontainer.scoreAlgaeAuto(()->"left"));

        routine.active().onTrue(Commands.sequence(
                startToAlgae1.resetOdometry(),
                startToAlgae1.spawnCmd()));

        startToAlgae1.done().onTrue(Commands.sequence(
                Commands.waitUntil(algae.limitHit()).withTimeout(3),
                algae1ToScore.resetOdometry(),
                algae1ToScore.spawnCmd()));

        algae1ToScore.done().onTrue(Commands.sequence(
                Commands.waitUntil(() -> superstructure.getState().equals(States.resting)),
                Commands.waitSeconds(0.5),
                scoreToAlgae2.spawnCmd()));

        scoreToAlgae2.done().onTrue(Commands.sequence(
                Commands.waitUntil(algae.limitHit()).withTimeout(3),
                algae2ToScore.resetOdometry(),
                algae2ToScore.spawnCmd()));

        algae2ToScore.done().onTrue(Commands.sequence(
            Commands.waitUntil(() -> superstructure.getState().equals(States.resting)),
            Commands.waitSeconds(0.5),
            scoreToAlgae3.spawnCmd()));

        scoreToAlgae3.done().onTrue(Commands.sequence(
                Commands.waitUntil(algae.limitHit()).withTimeout(3),
                algae3ToScore.resetOdometry(),
                algae3ToScore.spawnCmd()));

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
                robotcontainer.collectAlgae(States.algaeFromReefLow, ()->"right"),
                algae1ToScore.resetOdometry(),
                algae1ToScore.spawnCmd()));

        algae1ToScore.done().onTrue(Commands.sequence(
                robotcontainer.scoreAlgae(States.algaeToBardge, ()->"left"),
                scoreToSafe.spawnCmd()));

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
            // scoreCoral(),
            Commands.waitSeconds(0.05),
            score1ToCollect.spawnCmd()
                .alongWith(robotcontainer.collectCoral(States.coralFromHp))));
        
        routine.observe(algae.currentHit()).onTrue(
            collectToscore2.spawnCmd());

        collectToscore2.atTimeBeforeEnd(0.3).onTrue(robotcontainer.scoreCoralAuto());

        collectToscore2.done().onTrue(Commands.sequence(
            // scoreCoralAuto(),
            Commands.waitUntil(() -> superstructure.getState().equals(States.resting)),
            score2ToCollect.spawnCmd()
                .alongWith(robotcontainer.collectCoral(States.coralFromHp))));

        return routine;
    }

}
