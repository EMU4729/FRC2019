package org.usfirst.frc4729.FRC2019;

public class Util {
    public static double linear(double from, double to, double minPower, double maxPower, double slowRange, double stopRange) {
        double difference = to - from;
        double direction = Math.signum(difference);
        difference = Math.abs(difference);
        if (difference < stopRange) {
            return 0;
        } else if (difference < slowRange) {
            return ((difference / slowRange) * (maxPower - minPower) + minPower) * direction;
        } else {
            return direction * maxPower;
        }
    }

    public static double normAngle(double angle) {
        return angle % 360;
    }
}