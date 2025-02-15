package frc.robot;

public class Constants {
    public static enum States {
        algaeFromGround(-0.0732, 0.36), // has left/right
        algaeFromReefLow(0.14, 0.09), // has left/right
        algaeFromReefHigh(0.14, 0.5), // has left/right
        algaeToBardge(0.15, 1.45), // has left/right 0.1674722
        algaeToProcessor(0.000, 0.19), // has left/right
        coralReefL2(0.25, 0.1), // not final
        coralReefL3(0.25, 0.1), // not final
        coralReefL4(0.25, 0.1), // not final
        coralHumanPlayer(0.25, 0.1), // not final
        resting(0.25, 0.01);

        public double armAngle;
        public double elevatorHeight;

        States(double angle, double height) {
            armAngle = angle;
            elevatorHeight = height;
        }
    }
}
