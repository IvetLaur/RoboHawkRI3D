package org.firstinspires.ftc.teamcode.util;

/**
 * Distance -> RPM table with straight-line interpolation between points.
 *
 * You test shots at a few distances (say 36", 48", 60") and write down the RPM
 * that scored. For 54" the table draws a straight line between the 48" and 60"
 * points. Outside the table it uses the nearest end value (never guesses wildly).
 */
public class LookupTable {

    private final double[] x;
    private final double[] y;

    /** @param x inputs, in increasing order   @param y outputs, same length */
    public LookupTable(double[] x, double[] y) {
        if (x.length != y.length || x.length < 2) {
            throw new IllegalArgumentException("Need at least 2 points and equal-length arrays");
        }
        for (int i = 1; i < x.length; i++) {
            if (x[i] <= x[i - 1]) throw new IllegalArgumentException("x must increase");
        }
        this.x = x.clone();
        this.y = y.clone();
    }

    public double get(double input) {
        if (input <= x[0]) return y[0];
        int last = x.length - 1;
        if (input >= x[last]) return y[last];
        int i = 0;
        while (input > x[i + 1]) i++;
        double t = (input - x[i]) / (x[i + 1] - x[i]);
        return y[i] + t * (y[i + 1] - y[i]);
    }
}
