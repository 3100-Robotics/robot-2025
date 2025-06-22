package frc.robot.math;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class Point {
    public double x;
    public double y;

    public Point(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Point() {
        this.x = 0;
        this.y = 0;
    }

    public Point(Pose2d pose) {
        x = pose.getX();
        y = pose.getY();
    }


    public Point add(double value) {
        return new Point(x+value, y+value);
    }

    public Point add(Point point2) {
        return new Point(x+point2.x, y+point2.y);
    }

    public Point sub(double value) {
        return new Point(x-value, y-value);
    }

    public Point sub(Point point2) {
        return new Point(x-point2.x, y-point2.y);
    }

    public Point mul(double value) {
        return new Point(x*value, y*value);
    }

    public static Point maxpoint(Point A, Point B, double f) {
        return new Point(f*B.x-A.x,f*B.x-A.x);
    }

    public static Point midpoint(Point A, Point B, double f) {
        return new Point((A.x+B.x)/2,(A.x+B.x)/2);
    }


    public Pose2d asPose2d() {
        return new Pose2d(x, y, new Rotation2d());
    }

    public double cross(Point B) {
        return -B.x * y + B.y * x;
    }

    public static Boolean isAleftB(Point A, Point B) {
        return A.cross(B)<0;
    }


    @Override
    public String toString() {
        return String.format("Point(%s, %s)", Math.floor(x * 1000) / 1000, Math.floor(y * 1000) / 1000);
    }
}
