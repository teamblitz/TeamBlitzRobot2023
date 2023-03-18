package frc.lib.math;

public class Angles {
    private Angles() {}

    public static double wrapAngle(double degrees) {
        double angle = degrees % 360;
        if (angle > 180) {
            angle -= 360;
        } else if (angle < -180) {
            angle += 360;
        }
        return angle;
    }
}
