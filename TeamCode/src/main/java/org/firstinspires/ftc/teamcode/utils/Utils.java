package org.firstinspires.ftc.teamcode.utils;

public class Utils {
    public static final double PI = Math.PI;
    public static final double TWO_PI = 2 * PI;

    public static double clamp(double x, double min, double max) {
        return Math.max(min, Math.min(max, x));
    }

    public static double reverseClamp(double x, double min, double max) {
        if (min < x && x < 0) {
            return min;
        } else if (0 < x && x < max) {
            return max;
        }
        return x;
    }

    public static double angleRange(double theta) {
        theta -= (TWO_PI) * Math.floor((theta + PI) / TWO_PI);
        return theta;
    }

    public static double angleDiff(double theta, double phi) {
        theta = angleRange(theta);
        phi = angleRange(theta);
        double diff = angleRange(theta - phi);
        return diff;
    }
}
