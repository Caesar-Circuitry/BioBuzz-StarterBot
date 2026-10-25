package org.firstinspires.ftc.teamcode.sim.biobuzz;

import java.util.ArrayList;
import java.util.List;

/**
 * Starting positions measured from AdvantageScope's official 2026-2027 field
 * model (am-5850 BIOBUZZ), converted to FTCodeSim/Pedro inches.
 *
 * Conversion: AdvantageScope draws the field model rotated 90 degrees from
 * logged FTC data, which works out to pedro = model + 70.75 in on both axes
 * (z is unchanged). The robot preloads and staged nectar that the model shows
 * outside the walls are not included.
 */
public final class FieldReference {
    private FieldReference() {}

    /** {x, y, z} of each starting piece on the field. */
    public static final double[][] RED_NECTAR_IN_CELL = {
            {49.76, 61.24, 49.94}, {53.38, 61.24, 49.94}, {57.00, 61.24, 49.94},
    };
    public static final double[][] BLUE_NECTAR_IN_CELL = {
            {84.52, 80.26, 49.94}, {88.14, 80.26, 49.94}, {91.76, 80.26, 49.94},
    };
    public static final double[][] GARDEN_POLLEN = {
            {1.48, 1.48, 1.40}, {4.37, 1.48, 1.40}, {7.26, 1.48, 1.40}, {10.15, 1.48, 1.40},
            {140.02, 140.02, 1.40}, {137.13, 140.02, 1.40}, {134.24, 140.02, 1.40}, {131.35, 140.02, 1.40},
    };
    public static final double[][] FLOWER_POLLEN = {
            {2.71, 47.36, 1.40}, {2.71, 47.36, 4.29}, {2.71, 47.36, 7.18}, {2.71, 47.36, 10.07},
            {47.36, 138.79, 1.40}, {47.36, 138.79, 4.29}, {47.36, 138.79, 7.18}, {47.36, 138.79, 10.07},
            {94.14, 2.71, 1.40}, {94.14, 2.71, 4.29}, {94.14, 2.71, 7.18}, {94.14, 2.71, 10.07},
            {138.79, 94.14, 1.40}, {138.79, 94.14, 4.29}, {138.79, 94.14, 7.18}, {138.79, 94.14, 10.07},
    };

    /** Cell centres (outer box) in the starting pose: {x, y, z}. */
    public static final double[] RED_RAISED_CELL = {58.01, 61.97, 55.00};
    public static final double[] RED_LOWERED_CELL = {58.01, 84.71, 41.87};
    public static final double[] BLUE_RAISED_CELL = {83.51, 79.53, 55.00};
    public static final double[] BLUE_LOWERED_CELL = {83.51, 56.79, 41.87};

    /** Problems where the sim's starting layout is more than {@code tolerance} inches off the model. */
    public static List<String> compare(BioBuzzField field, double tolerance) {
        List<String> problems = new ArrayList<>();
        matchPieces(problems, field.redNectar, RED_NECTAR_IN_CELL, "red nectar in its cell", tolerance);
        matchPieces(problems, field.blueNectar, BLUE_NECTAR_IN_CELL, "blue nectar in its cell", tolerance);
        matchPieces(problems, field.pollen, GARDEN_POLLEN, "garden pollen", tolerance);
        matchPieces(problems, field.pollen, FLOWER_POLLEN, "flower pollen", tolerance);

        Hive red = field.hives.get(Alliance.RED), blue = field.hives.get(Alliance.BLUE);
        matchPoint(problems, red.cellCenter(red.upSide()), RED_RAISED_CELL, "red raised cell", tolerance);
        matchPoint(problems, red.cellCenter(-red.upSide()), RED_LOWERED_CELL, "red lowered cell", tolerance);
        matchPoint(problems, blue.cellCenter(blue.upSide()), BLUE_RAISED_CELL, "blue raised cell", tolerance);
        matchPoint(problems, blue.cellCenter(-blue.upSide()), BLUE_LOWERED_CELL, "blue lowered cell", tolerance);
        return problems;
    }

    private static void matchPieces(List<String> problems, List<Ball> balls, double[][] reference, String what, double tol) {
        for (double[] ref : reference) {
            double best = Double.MAX_VALUE;
            for (Ball b : balls) {
                if (b.state == Ball.State.STAGED || b.state == Ball.State.IN_ROBOT) continue;
                best = Math.min(best, Math.sqrt(sq(b.x - ref[0]) + sq(b.y - ref[1]) + sq(b.z - ref[2])));
            }
            if (best > tol) {
                problems.add(String.format("no %s within %.1f in of the field model's (%.1f, %.1f, %.1f); nearest is %.1f in away",
                        what, tol, ref[0], ref[1], ref[2], best));
            }
        }
    }

    private static void matchPoint(List<String> problems, double[] sim, double[] ref, String what, double tol) {
        double d = Math.sqrt(sq(sim[0] - ref[0]) + sq(sim[1] - ref[1]) + sq(sim[2] - ref[2]));
        if (d > tol) {
            problems.add(String.format("%s is %.1f in from the field model: sim (%.1f, %.1f, %.1f) vs model (%.1f, %.1f, %.1f)",
                    what, d, sim[0], sim[1], sim[2], ref[0], ref[1], ref[2]));
        }
    }

    private static double sq(double v) { return v * v; }
}
