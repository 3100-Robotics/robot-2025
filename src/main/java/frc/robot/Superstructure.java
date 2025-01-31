package frc.robot;

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
import frc.robot.Constants.States;
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
        SmartDashboard.putNumber("arm pos", Units.rotationsToDegrees(arm.getPosition()));
    }

    private Command goTo(States state, String leftRight) { // TODO: make left/right function
        if (currentState == States.resting || currentState == state) {
            return Commands.sequence(
                    runOnce(() -> currentState=state),
                    Commands.parallel(
                    elevator.goToPos(state.elevatorHeight),
                    arm.goToPos(leftRight.equals("right") ? state.armAngle : 0.5-state.armAngle)));
        }
        else {
            return Commands.sequence(
                    runOnce(() -> currentState=state),
                    Commands.parallel(
                            elevator.goToPos(States.resting.elevatorHeight),
                            arm.goToPos(States.resting.armAngle)),
                    Commands.parallel(
                            elevator.goToPos(state.elevatorHeight),
                            arm.goToPos(leftRight.equals("right") ? state.armAngle : 0.5-state.armAngle)));
        }
    }

    public Command goToPos(States desiredState, String side) {
        return defer(() -> goTo(desiredState, side));
    }
}
