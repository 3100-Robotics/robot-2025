package frc.robot;

public class Constants {
    public static enum States {
        algaeFromGround(-0.0732, 0.36), // has left/right
        algaeFromLollipop(0.05, 0), // has left/right
        algaeFromReefLow(0.14, 0.09), // has left/right
        algaeFromReefHigh(0.14, 0.5), // has left/right
        algaeToBardge(0.15, 1.45), // has left/right 0.1674722
        algaeToProcessor(0.000, 0.19), // has left/right
        resting(0.25, 0.015);

        public double armAngle;
        public double elevatorHeight;

        States(double angle, double height) {
            armAngle = angle;
            elevatorHeight = height;
        }
    }
}
