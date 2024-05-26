package org.firstinspires.ftc.teamcode.centerstage.util;

public class MathUtil {

    public static double clamp(double value, double min, double max) {
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return Math.max(Math.min(value, max), min);
    }

    public static int clamp(int value, int min, int max) {
        if (min > max) {
            int tempMin = min;
            min = max;
            max = tempMin;
        }
        return Math.max(Math.min(value, max), min);
    }

    public static boolean inRange(double value, double min, double max) {
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return value > min && value < max;
    }

    public static boolean outOfRange(double value, double min, double max) {
        if (min > max) {
            double tempMin = min;
            min = max;
            max = tempMin;
        }
        return value <= min || value >= max;
    }

    public static boolean approximately(double value, double targetValue, double approximation) {
        return value > (targetValue - approximation) && value < (targetValue + approximation);
    }

    public static double map(double x, double minX, double maxX, double minY, double maxY) {
        if (maxX == minX) {
            return 0;
        }
        // Scale the value to a 0-1 range based on its position in the original range
        double normalizedValue = (x - minX) / (maxX - minX);

        // Map the normalized value to the new range
        return normalizedValue * (maxY - minY) + minY;
    }

}
