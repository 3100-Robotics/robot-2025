package frc.robot;

import java.util.List;
import java.util.Map;

public class Constants {

    public static class superstructureConstants {
        public static enum States {
            algaeFromGround(-0.0702, 0.3, true), // has left/right
            algaeFromLollipop(0.05, 0, false), // has left/right
            algaeFromReefLow(0.14, 0.05, false), // has left/right
            algaeFromReefHigh(0.15, 0.45, 0.1, false), // has left/right
            algaeToBardge(0.15, 1.45, true), // has left/right 0.1674722
            algaeToProcessor(-0.07, 0.59, true), // has left/right 0.000, 0.19 //// -0.07, 0.49
            coralFromGround(-0.022, 0.14, true), // no left/right
            coralFromHp(0.2, 0.18, false), // no left/right
            coralToL1(0.15, 0.01, false), // no left/right
            resting(0.24, 0.015, false);
    
            public double armAngle1;
            public double elevatorHeight1;
            public double elevatorHeightTrigger;
    
            public Boolean elevatorFirst1;
    
    
            States(double angle, double height, Boolean elevatorFirst) {
                armAngle1 = angle;
                elevatorHeight1 = height;
                elevatorHeightTrigger = height;
                elevatorFirst1 = elevatorFirst;
            }
    
            States(double angle1, double height1, double armTriggerHeight, Boolean elevatorFirstOne) {
                armAngle1 = angle1;
                elevatorHeight1 = height1;
                elevatorHeightTrigger = armTriggerHeight;
                elevatorFirst1 = elevatorFirstOne;
            }
        }

        public static Map<States, List<States>> allowedToFroms = Map.of(
            States.algaeFromGround, List.of(States.resting, States.coralFromGround, States.algaeFromLollipop),
            States.algaeFromLollipop, List.of(States.resting, States.algaeFromGround, States.coralFromGround),
            States.algaeFromReefHigh, List.of(States.resting),
            States.algaeFromReefLow, List.of(States.resting, States.coralToL1),
            States.algaeToBardge, List.of(States.resting),
            States.algaeToProcessor, List.of(States.resting),
            States.coralFromGround, List.of(States.resting, States.algaeFromGround, States.algaeFromLollipop),
            States.coralFromHp, List.of(States.resting),
            States.coralToL1, List.of(States.resting, States.algaeFromReefLow),
            States.resting, List.of(States.resting));
    }
}
