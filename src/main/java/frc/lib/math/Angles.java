package frc.lib.math;

public class Angles {
    private Angles() {}

    public static double wrapAngle180(double degrees) {
        return wrapAngle(degrees, -180, 180);
    }

    public static double wrapAngle(double degrees, double min, double max) {
        double angle = degrees % 360;
        if (angle > max) {
            angle -= 360;
        } else if (angle < min) {
            angle += 360;
        }
        return angle;
    }
}
