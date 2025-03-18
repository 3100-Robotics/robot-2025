package frc.robot;

public class Constants {
    public static enum States {
        algaeFromGround(-0.0732, 0.33), // has left/right
        algaeFromLollipop(0.05, 0), // has left/right
        algaeFromReefLow(0.14, 0.09), // has left/right
        algaeFromReefHigh(0.14, 0.5), // has left/right
        algaeToBardge(0.15, 1.45), // has left/right 0.1674722
        algaeToProcessor(0.000, 0.19), // has left/right
        coralFromGround(-0.022, 0.14), // no left/right -0.0432
        coralFromHp(0.2, 0.18), // no left/right
        coralToL1(0.15, 0.01), // no left/right
        resting(0.24, 0.015);

        public double armAngle;
        public double elevatorHeight;

        States(double angle, double height) {
            armAngle = angle;
            elevatorHeight = height;
        }
    }
}
