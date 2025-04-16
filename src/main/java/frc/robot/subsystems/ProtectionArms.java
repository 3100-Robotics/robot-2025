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

    private double leftrest = 95;
    private double rightrest = 95;
    private double leftdown = 0;
    private double rightdown = 160;

    public Command restArm() {
        return Commands.runOnce(() -> {
            motorLeft.setAngle(leftrest);
            motorRight.setAngle(rightrest);
        });
    }

    public Command set(String leftright){
        return Commands.sequence(restArm(), Commands.runOnce(() -> {
            switch (leftright) {
                case "left":
                    motorLeft.setAngle(leftdown);
                    break;
                case "right":
                    motorRight.setAngle(rightdown);
                    break;
                default:
                    DataLogManager.log("leftright needs to be either left or right");
            }
        }));
    }
}
