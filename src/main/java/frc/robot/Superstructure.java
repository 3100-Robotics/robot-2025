package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Elevator;

public class Superstructure implements Subsystem{
    final double armLength = 0.865;
    
    // Trigger forwardArmLimit;
    // Trigger backwardArmLimit;

    private Elevator elevator;
    private Arm arm;

    private Mechanism2d mech = new Mechanism2d(2, 8);
    private MechanismRoot2d root = mech.getRoot("elevator", 0, 0);
    private MechanismLigament2d elevatorMech;
    private MechanismLigament2d armMech;

    public Superstructure(Elevator elevator, Arm arm) {
        this.elevator = elevator;
        this.arm = arm;

        elevatorMech = root.append(new MechanismLigament2d("elevator", 0.9, 90));
        armMech = elevatorMech.append(
            new MechanismLigament2d("arm", armLength, 90, 6, new Color8Bit(Color.kAqua)));
        

        // Rectangle2d driveTrainRect = new Rectangle2d(new Pose2d(0, 0, new Rotation2d()), 1, 0.2);
        // Rectangle2d climberRect = new Rectangle2d(new Pose2d(0, 0.25, new Rotation2d()), 1, 0.5);

        // forwardArmLimit = new Trigger(() -> {
        //     Translation2d pos = new Translation2d(
        //         armLength*Math.cos(arm.getPosition()), 
        //         elevator.getHeight()-armLength*Math.sin(arm.getPosition()));
        //     return !(driveTrainRect.contains(pos) && climberRect.contains(pos));
        // });
    }

    @Override
    public void simulationPeriodic() {
        SmartDashboard.putData(mech);
    }

    public Command goToPos(double height, double angle) {
        return elevator.goToPos(0, null, null);
    }
}
