package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.superstructureConstants.States;
import frc.robot.Constants.superstructureConstants;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class Superstructure extends SubsystemBase {
    final double armLength = 0.865;

    private Elevator elevator;
    private Arm arm;

    private States currentState = States.resting;

    private Mechanism2d mech = new Mechanism2d(2, 3);
    private MechanismRoot2d root = mech.getRoot("elevator", 1, 0);
    private MechanismLigament2d elevatorMech;
    private MechanismLigament2d armMech;

    public Superstructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;

        elevatorMech = root.append(new MechanismLigament2d("elevator", 0.9, 90));
        armMech = elevatorMech.append(
            new MechanismLigament2d("arm", armLength, 90, 6, new Color8Bit(Color.kAqua)));
        
        SmartDashboard.putData("elevator + arm", mech);
    }

    @Override
    public void periodic() {
        // System.out.println(arm.getPosition());
        elevatorMech.setLength(elevator.getHeight());
        armMech.setAngle(Units.rotationsToDegrees(arm.getPosition())-90);
        SmartDashboard.putString("elevator state", getState().name());
    }

    private Command goTo(States state, String leftRight) {
        boolean yesGoToStatic = true;
        Supplier<Command> goToStatic = () -> Commands.sequence(
                Commands.runOnce(() -> currentState=state),
                Commands.parallel(
                    elevator.goToPos(States.resting.elevatorHeight),
                    arm.goToPos(States.resting.armAngle)));

        Supplier<Command> goToElevatorFirst = () -> Commands.sequence(
            Commands.runOnce(() -> currentState=state),
            elevator.goToPos(state.elevatorHeight),
            arm.goToPos(leftRight.equals("right") ? 0.478-state.armAngle : state.armAngle));

        Supplier<Command> goToArmDelayed = () -> Commands.sequence(
            Commands.runOnce(() -> currentState=state),
            Commands.parallel(
                elevator.goToPos(state.elevatorHeight),
                Commands.sequence(
                    Commands.waitUntil(() -> elevator.getHeight() >= state.elevatorHeightTrigger),
                    arm.goToPos(leftRight.equals("right") ? 0.478-state.armAngle : state.armAngle))));

        Supplier<Command> goTo = () -> Commands.sequence(
            Commands.runOnce(() -> currentState=state),
            Commands.parallel(
                elevator.goToPos(state.elevatorHeight),
                arm.goToPos(leftRight.equals("right") ? 0.478-state.armAngle : state.armAngle)));
        
        if (superstructureConstants.allowedToFroms.get(currentState).contains(state) || currentState.equals(States.resting) || currentState.equals(state)) {
            yesGoToStatic = false;
        }

        return Commands.sequence(
            yesGoToStatic ? goToStatic.get() : Commands.none(),
            (state.elevatorFirst ? goToElevatorFirst.get() : 
            !(state.elevatorHeightTrigger == state.elevatorHeight) ? goToArmDelayed.get() : goTo.get()));
    }

    public Command goToPos(States desiredState, String side) {
        return defer(() -> goTo(desiredState, side));
    }

    public Trigger atSetpoint() {
        return elevator.atSetpoint().and(arm.atSetpoint());
    }

    public States getState() {
        return currentState;
    }
}
