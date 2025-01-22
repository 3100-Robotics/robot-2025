package frc.robot;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;

public class Superstructure {
    
    private Supplier<Command> elevatorController;
    private Supplier<Command> armController;

    public Superstructure(Supplier<Command> elevatorControl, Supplier<Command> armControl) {
        elevatorController = elevatorControl;
        armController = armControl;
    }

    public Command goToPos(double height, double angle) {
        return elevatorController.get();
    }
}
