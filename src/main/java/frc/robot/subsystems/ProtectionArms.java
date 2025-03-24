package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class ProtectionArms extends SubsystemBase {  
    private Servo motorLeft = new Servo(9);
    private Servo motorRight = new Servo(8);

    @Override
    public void periodic() {
        SmartDashboard.putNumber("armleftangle", motorLeft.getAngle());
        SmartDashboard.putNumber("armleftangle", motorRight.getAngle());
    }

    public Command restArm(){
        return Commands.runOnce(() -> {
            motorLeft.setAngle(90);
            motorRight.setAngle(90);
        });
    }

    public Command set(String leftright){
        return Commands.sequence(restArm(), Commands.runOnce(() -> {
            switch (leftright) {
                case "left":
                    motorLeft.setAngle(180);
                    break;
                case "right":
                    motorRight.setAngle(0);
                    break;
                default:
                    DataLogManager.log("leftright needs to be either left or right");
            }
        }));
    }
}
