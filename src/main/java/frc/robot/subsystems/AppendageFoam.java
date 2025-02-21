package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class AppendageFoam extends SubsystemBase {
    private Servo motor = new Servo(9);

    public Command setAngle(double angle) {
        return runOnce( ()->{
            motor.setAngle(angle);
        } );
    }
}
