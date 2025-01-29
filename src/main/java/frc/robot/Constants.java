package frc.robot;

public class Constants {
    public static enum States {
        algaeFromGround(-0.0732, 0.3302), // has left/right
        algaeFromReefLow(0, 0.711), // has left/right
        algaeFromReefHigh(0, 1.11), // has left/right
        algaeToBardge(1.9, 1.85), // has left/right
        algaeToProcessor(0.030, 0.25), // has left/right
        coralReefL2(),
        coralReefL3(),
        coralReefL4(),
        coralHumanPlayer(),
        resting(0.25, 0.1);

        public double armAngle;
        public double elevatorHeight;

        States(double angle, double height) {
            armAngle = angle;
            elevatorHeight = height;
        }
    }
}
