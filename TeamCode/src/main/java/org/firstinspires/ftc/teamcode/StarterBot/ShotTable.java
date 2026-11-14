package org.firstinspires.ftc.teamcode.StarterBot;

public class ShotTable {

    private static final double[] DISTANCES_IN = {
            24, 28, 32, 36, 40, 44, 48, 52, 56, 60, 64, 68, 72, 76, 80
    };
    private static final double[] SPEEDS = {
            1180, 1110, 1100, 1100, 1120, 1140, 1160, 1180, 1200, 1230, 1260, 1290, 1310, 1340, 1360
    };

    public static boolean isValid() {
        return DISTANCES_IN.length == SPEEDS.length && DISTANCES_IN.length >= 2;
    }

    public static double speedFor(double distanceIn, double fallback) {
        if (!isValid()) return fallback;

        int last = DISTANCES_IN.length - 1;
        if (distanceIn <= DISTANCES_IN[0]) return SPEEDS[0];
        if (distanceIn >= DISTANCES_IN[last]) return SPEEDS[last];

        for (int i = 0; i < last; i++) {
            if (distanceIn <= DISTANCES_IN[i + 1]) {
                double t = (distanceIn - DISTANCES_IN[i]) / (DISTANCES_IN[i + 1] - DISTANCES_IN[i]);
                return SPEEDS[i] + t * (SPEEDS[i + 1] - SPEEDS[i]);
            }
        }
        return SPEEDS[last];
    }
//i suck at these status crap. ai thank you for all my telemetry & panels addData
    public static String status() {
        return isValid() ? "OK (" + DISTANCES_IN.length + " points)"
                : "BROKEN: " + DISTANCES_IN.length + " distances but " + SPEEDS.length + " speeds";
    }
}