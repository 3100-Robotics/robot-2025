package frc.robot.math;

import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import frc.robot.Constants;
import frc.robot.Constants.LocationsFake;

// double rcord[] = {normalrobot.x, normalrobot.y};
// double vec[] = {Math.cos(Math.toRadians(three60rot)), Math.sin(Math.toRadians(three60rot))};
// double pointsr[][] = {rcord, vec};
// jsonObject.put("robot", pointsr);

// double down[] = {LocationsFake.REEF_DOWNLEFT.x, LocationsFake.REEF_DOWNLEFT.y};
// double left[] = {LocationsFake.REEF_LEFT.x, LocationsFake.REEF_LEFT.y};
// double up[] = {LocationsFake.REEF_UP.x, LocationsFake.REEF_UP.y};
// double right[] = {LocationsFake.REEF_RIGHT.x, LocationsFake.REEF_RIGHT.y};
// double reef_points[][] = {down,left,up,right};
// jsonObject.put("reef_points", reef_points);

// double seven[] = {LocationsFake.SEVEN.x, LocationsFake.SEVEN.y};
// double three[] = {LocationsFake.THREE.x, LocationsFake.THREE.y};
// double one[] = {LocationsFake.ONE.x, LocationsFake.ONE.y};
// double reef_sides[][] = {seven,three,one};
// jsonObject.put("reef_sides", reef_sides);


// SmartDashboard.putString("qdbpoints", jsonObject.toString());

/* Takes robot pose and provide
    useful information regarding Reefscape's
    various field elements

    Reef sidecodes coorespond to the april tags as follows:
         April tags
      |--------------
      |    |  B |  R
      |----|----|----
    S | 00 | 21 | 07
    i |----|----|----
    d | 01 | 20 | 08
    e |----|----|----
    c | 03 | 19 | 09
    o |----|----|----
    d | 04 | 22 | 06
    e |----|----|----
      | 06 | 17 | 11
      |----|----|----
      | 07 | 18 | 10
    
*/

public class LocatorEngine {
    private class Telemetry {
        private final NetworkTableInstance inst = NetworkTableInstance.getDefault();
        private final NetworkTable qdbTable = inst.getTable("qdb");
        private final BooleanEntry static_points_valid = inst.getBooleanTopic(".static_points_valid").getEntry(false);

        private final NetworkTable robot = qdbTable.getSubTable("robot");
            private final DoubleArrayPublisher location = robot.getDoubleArrayTopic("location").publish();
            private final DoubleArrayPublisher vec = robot.getDoubleArrayTopic("vec").publish();

        private final NetworkTable reef_points = qdbTable.getSubTable("reef_points");
            private final DoubleArrayPublisher down = reef_points.getDoubleArrayTopic("down").publish();
            private final DoubleArrayPublisher left = reef_points.getDoubleArrayTopic("left").publish();
            private final DoubleArrayPublisher up = reef_points.getDoubleArrayTopic("up").publish();
            private final DoubleArrayPublisher right = reef_points.getDoubleArrayTopic("right").publish();

        private final NetworkTable reef_sides = qdbTable.getSubTable("reef_sides");
            private final DoubleArrayPublisher seven = reef_sides.getDoubleArrayTopic("seven").publish();
            private final DoubleArrayPublisher three = reef_sides.getDoubleArrayTopic("three").publish();
            private final DoubleArrayPublisher one = reef_sides.getDoubleArrayTopic("one").publish();
    }

    private Telemetry telemetry = new Telemetry();

    // Relevant calculation lambdas
    public Supplier<Pose2d> getRobotPose;
    public Supplier<Point> getRobotPositon = () -> new Point(getRobotPose.get().getX(), getRobotPose.get().getY());
    public Supplier<Boolean> isRobotOnBlue = () -> (getRobotPositon.get().x < 8.75);

    public Supplier<Point> reef_location = () -> isRobotOnBlue.get() ? Constants.LocationsFake.BLUE_REEF : Constants.LocationsFake.RED_REEF;
    public Supplier<Point> getRobotNormalized = () -> getRobotPositon.get().sub(reef_location.get());

    public Supplier<Double> getRobotRotation = () -> getRobotPose.get().getRotation().getDegrees();
    public Supplier<Double> getRobotRotation360 = () -> getRobotRotation.get() < 0 ? getRobotRotation.get()+360 : getRobotRotation.get();

    public Integer dirty = 0;

    public LocatorEngine(Supplier<Pose2d> getRobotPose) {
        this.getRobotPose = getRobotPose;
    }

    public Boolean reefSideLeft() { // Error bit 0
        double rot = getRobotRotation360.get();

        // Which reef to localize against
        // Point reef_location = isRobotOnBlue.get() ? Constants.LocationsFake.BLUE_REEF : Constants.LocationsFake.RED_REEF;

        // Point normalrobot = getRobotPositon.get().sub(reef_location); // Robot relative ot reaf

        int sidecode = // TODO: Find an elagent way to document sidecodes
            ((Point.isAleftB(getRobotNormalized.get(), Constants.LocationsFake.REEF_LEFT)?1:0)<<2) +
            ((Point.isAleftB(getRobotNormalized.get(), Constants.LocationsFake.REEF_UP)?1:0)<<1) +
             (Point.isAleftB(getRobotNormalized.get(), Constants.LocationsFake.REEF_RIGHT)?1:0);

        Point sidevector = Constants.LocationsFake.SEVEN;
        switch (sidecode) {
        case 0: case 7:
            sidevector = Constants.LocationsFake.SEVEN;
            break;
        case 4: case 3:
            sidevector = Constants.LocationsFake.THREE;
            break;
        case 6: case 1:
            sidevector = Constants.LocationsFake.ONE;
            break;
        default:
            DataLogManager.log("Catastrophic Error has occured in side detection for reef! Report to even ASAP"); // TODO: Dashboard?
            dirty |= 0b1;
            break;
        }

        Boolean shouldLeft = Point.isAleftB(new Point(Math.cos(Math.toRadians(rot)), Math.sin(Math.toRadians(rot))), sidevector);
        shouldLeft = sidecode % 2 == 0 ? !shouldLeft : shouldLeft; // If we're on the ev*n half invert the side

        return shouldLeft;
    }

    public Boolean bargeSideLeft() { // Error bit 1
        double rot = getRobotRotation.get();
        Boolean sidepointingblue = (0 < rot && rot < 180);
        // The bitshifting amalgamation here takes the sideisblue and sidepointblue
        // and encodes them into a single two bit number
        // Then it is determined weather the robot would want to push
        // algae left or right
        switch (((isRobotOnBlue.get()?1:0)<<1) + (sidepointingblue?1:0)) {
            case 0: case 3:
                return false; // Barge right
            case 1: case 2:
                return true; // Barge left
            default:
                System.out.println("Error"); // TODO: Learn how to log, also Dahboard?
                dirty |= 0b10;
                return false;
        }
    }

    public Boolean procSideLeft() { // Error bit 2
        double rot = getRobotRotation360.get()-90 % 360;
        Boolean sidepointingblueds = (0 < rot && rot < 180);
        boolean robotnearblueproc = getRobotPositon.get().y < 8.062/2;

        switch (((robotnearblueproc?1:0)<<1) + (sidepointingblueds?1:0)) {
            case 0: case 3:
                return true; // Barge right
            case 1: case 2:
                return false; // Barge left
            default:
                System.out.println("Error"); // TODO: Learn how to log, also Dahboard?
                dirty |= 0b100;
                return false;
        }
    }

    public void sendState() {
        telemetry.location.set(new double[]{getRobotNormalized.get().x, getRobotNormalized.get().y});
        telemetry.vec.set(new double[]{Math.cos(Math.toRadians(getRobotRotation360.get())), Math.sin(Math.toRadians(getRobotRotation360.get()))});

        if (!telemetry.static_points_valid.get()) {
            telemetry.down.set(new double[]{LocationsFake.REEF_DOWNLEFT.x, LocationsFake.REEF_DOWNLEFT.y});
            telemetry.left.set(new double[]{LocationsFake.REEF_LEFT.x, LocationsFake.REEF_LEFT.y});
            telemetry.up.set(new double[]{LocationsFake.REEF_UP.x, LocationsFake.REEF_UP.y});
            telemetry.right.set(new double[]{LocationsFake.REEF_RIGHT.x, LocationsFake.REEF_RIGHT.y});

            telemetry.seven.set(new double[]{LocationsFake.SEVEN.x, LocationsFake.SEVEN.y});
            telemetry.three.set(new double[]{LocationsFake.THREE.x, LocationsFake.THREE.y});
            telemetry.one.set(new double[]{LocationsFake.ONE.x, LocationsFake.ONE.y});
            telemetry.static_points_valid.set(true);
            telemetry.static_points_valid.unpublish();
        }
    }
}
