package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class Superstructure {
    
    Trigger forwardArmLimit;
    Trigger backwardArmLimit;

    private Elevator elevator;
    private Arm arm;

    public Superstructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;

        
    }

    public Command goToPos(double height, double angle) {
        return elevator.goToPos(0, null, null);
    }
}
