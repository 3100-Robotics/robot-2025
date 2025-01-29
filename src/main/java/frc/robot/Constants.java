package frc.robot;

public class Constants {
    public static enum States {
        algaeFromGround(-0.0732, 0.3302), // has left/right
        algaeFromReefLow(0, 0.711), // has left/right
        algaeFromReefHigh(0, 1.11), // has left/right
        algaeToBardge(0.1674722, 1.85), // has left/right
        algaeToProcessor(0.030, 0.25), // has left/right
        coralReefL2(0.25, 0.1), // not final
        coralReefL3(0.25, 0.1), // not final
        coralReefL4(0.25, 0.1), // not final
        coralHumanPlayer(0.25, 0.1), // not final
        resting(0.25, 0.1);

        public double armAngle;
        public double elevatorHeight;

        States(double angle, double height) {
            armAngle = angle;
            elevatorHeight = height;
        }
    }
}
