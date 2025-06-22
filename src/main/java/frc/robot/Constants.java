package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;


import frc.robot.math.Point;

public class Constants {
    public static double MakeFake = 8.062/317;

    public static class LocationsReal {
        public static Point BLUE_REEF = new Point(176.246, 158.007);
        public static Point RED_REEF = new Point(513.630, 157.999);

        public static Point REEF_UP = new Point(0, 53.976205);
        public static Point REEF_LEFT = new Point(-46.745550, 26.987649);
        public static Point REEF_RIGHT = new Point(46.745550, 26.987649);
        public static Point REEF_DOWNLEFT = new Point(-46.745550, -26.989463);

        public static Point SEVEN = new Point((REEF_DOWNLEFT.x+REEF_LEFT.x)/2,(REEF_DOWNLEFT.y+REEF_LEFT.y)/2);
        public static Point THREE = new Point((REEF_LEFT.x+REEF_UP.x)/2,(REEF_LEFT.y+REEF_UP.y)/2);
        public static Point ONE = new Point((REEF_UP.x+REEF_RIGHT.x)/2,(REEF_UP.y+REEF_RIGHT.y)/2);

        public static Point PROCBLUE = new Point(230.476, 0);
        public static Point PROCRED = new Point(449.899854, 317);
    }

    public static class LocationsFake {
        public static Point BLUE_REEF = LocationsReal.BLUE_REEF.mul(MakeFake);
        public static Point RED_REEF = LocationsReal.RED_REEF.mul(MakeFake);

        public static Point REEF_UP = LocationsReal.REEF_UP.mul(MakeFake);
        public static Point REEF_LEFT = LocationsReal.REEF_LEFT.mul(MakeFake);
        public static Point REEF_RIGHT = LocationsReal.REEF_RIGHT.mul(MakeFake);
        public static Point REEF_DOWNLEFT = LocationsReal.REEF_DOWNLEFT.mul(MakeFake);

        public static Point SEVEN = LocationsReal.SEVEN.mul(MakeFake);
        public static Point THREE = LocationsReal.THREE.mul(MakeFake);
        public static Point ONE = LocationsReal.ONE.mul(MakeFake);

        public static Point PROCBLUE = LocationsReal.PROCBLUE.mul(MakeFake);
        public static Point PROCRED = LocationsReal.PROCRED.mul(MakeFake);
    }

    public static class superstructureConstants {
        public static enum States {
            algaeFromGround(-0.0702, 0.3, true), // has left/right
            algaeFromLollipop(0.05, 0, false), // has left/right
            algaeFromReefLow(0.14, 0.06, false), // has left/right
            algaeFromReefHigh(0.15, 0.45, 0.01, false), // has left/right
            algaeToBardge(0.15, 1.55, true), // has left/right 0.1674722
            algaeToBardgeAuto(0.1, 1.65, 0, false),
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
