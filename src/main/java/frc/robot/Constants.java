package frc.robot;

import java.util.List;
import java.util.Map;

public class Constants {

    public static class superstructureConstants {
        public static enum States {
            algaeFromGround(-0.0702, 0.3, true), // has left/right
            algaeFromLollipop(0.05, 0, false), // has left/right
            algaeFromReefLow(0.14, 0.09, false), // has left/right
            algaeFromReefHigh(0.14, 0.4, 0.14, 0.5, true, false), // has left/right
            algaeToBardge(0.15, 1.45, true), // has left/right 0.1674722
            algaeToProcessor(0.000, 0.19, false), // has left/right
            coralFromGround(-0.022, 0.14, true), // no left/right
            coralFromHp(0.2, 0.18, false), // no left/right
            coralToL1(0.15, 0.01, false), // no left/right
            resting(0.24, 0.015, false);
    
            public double armAngle1;
            public double elevatorHeight1;
            public double armAngle2;
            public double elevatorHeight2;
    
            public Boolean elevatorFirst1;
            public Boolean elevatorFirst2;
    
    
            States(double angle, double height, Boolean elevatorFirst) {
                armAngle1 = angle;
                elevatorHeight1 = height;
                armAngle2 = angle;
                elevatorHeight2 = height;
                elevatorFirst1 = elevatorFirst;
                elevatorFirst2 = elevatorFirst;
            }
    
            States(double angle1, double height1, double angle2, double height2, Boolean elevatorFirstOne, Boolean elevatorFirstTwo) {
                armAngle1 = angle1;
                elevatorHeight1 = height1;
                armAngle2 = angle2;
                elevatorHeight2 = height2;
                elevatorFirst1 = elevatorFirstOne;
                elevatorFirst2 = elevatorFirstTwo;
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
