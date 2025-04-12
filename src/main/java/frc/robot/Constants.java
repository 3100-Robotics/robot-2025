package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class Constants {

    public static class superstructureConstants {
        public static enum States {
            algaeFromGround(-0.0702, 0.3, true), // has left/right
            algaeFromLollipop(0.05, 0, false), // has left/right
            algaeFromReefLow(0.14, 0.06, false), // has left/right
            algaeFromReefHigh(0.15, 0.45, 0.01, false), // has left/right
            algaeToBardge(0.15, 1.55, true), // has left/right 0.1674722
            algaeToBardgeAuto(0.1, 1.65, 1, false),
            algaeToProcessor(-0.07, 0.59, true), // has left/right 0.000, 0.19 //// -0.07, 0.49
            coralFromGround(-0.025, 0.14, true), // no left/right
            coralFromHp(0.2, 0.18, false), // no left/right
            coralToL1(0.15, 0.01, false), // no left/right
            rezeroElevator(0.24, -0.1, false), //no left/right
            resting(0.24, 0.015, false);
    
            public double armAngle;
            public double elevatorHeight;
            public double elevatorHeightTrigger;
    
            public Boolean elevatorFirst;
    
    
            States(double angle, double height, Boolean elevatorFirst) {
                armAngle = angle;
                elevatorHeight = height;
                elevatorHeightTrigger = height;
                this.elevatorFirst = elevatorFirst;
            }
    
            States(double angle, double height, double armTriggerHeight, Boolean elevatorFirstOne) {
                armAngle = angle;
                elevatorHeight = height;
                elevatorHeightTrigger = armTriggerHeight;
                elevatorFirst = elevatorFirstOne;
            }
        }

        public static Map<States, List<States>> allowedToFroms = new HashMap<States, List<States>>();
        
        static {
            allowedToFroms.put(States.algaeFromGround, List.of(States.resting, States.coralFromGround, States.algaeFromLollipop));
            allowedToFroms.put(States.algaeFromLollipop, List.of(States.resting, States.algaeFromGround, States.coralFromGround));
            allowedToFroms.put(States.algaeFromReefHigh, List.of(States.resting, States.algaeToBardgeAuto));
            allowedToFroms.put(States.algaeFromReefLow, List.of(States.resting, States.coralToL1));
            allowedToFroms.put(States.algaeToBardge, List.of(States.resting));
            allowedToFroms.put(States.algaeToBardgeAuto, List.of(States.resting));
            allowedToFroms.put(States.algaeToProcessor, List.of(States.resting));
            allowedToFroms.put(States.coralFromGround, List.of(States.resting, States.algaeFromGround, States.algaeFromLollipop));
            allowedToFroms.put(States.coralFromHp, List.of(States.resting));
            allowedToFroms.put(States.coralToL1, List.of(States.resting, States.algaeFromReefLow));
            allowedToFroms.put(States.rezeroElevator, List.of(States.resting));
            allowedToFroms.put(States.resting, List.of(States.resting));
        }
    }
}
