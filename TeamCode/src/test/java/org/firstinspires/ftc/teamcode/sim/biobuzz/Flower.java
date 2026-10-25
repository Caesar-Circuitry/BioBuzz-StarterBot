package org.firstinspires.ftc.teamcode.sim.biobuzz;

import java.util.ArrayList;
import java.util.List;

/** A tube on the perimeter wall. Balls drop in the top and stack up; robots pull from the bottom. */
public class Flower {
    public final int index;
    public final double x, y;
    /** Bottom of the stack is index 0. */
    public final List<Ball> stack = new ArrayList<>();
    private final BioBuzzConfig c;

    Flower(int index, double x, double y, BioBuzzConfig c) {
        this.index = index;
        this.x = x;
        this.y = y;
        this.c = c;
    }

    void arrange() {
        double z = c.flowerStackBaseHeight;
        for (Ball b : stack) {
            b.place(x, y, z + b.radius);
            z += 2 * b.radius;
        }
    }

    /** The alliance whose nectar is top-most owns the flower (null = nobody). */
    public Alliance owner() {
        for (int i = stack.size() - 1; i >= 0; i--) {
            Alliance a = Alliance.ofNectar(stack.get(i).kind);
            if (a != null) return a;
        }
        return null;
    }

    /** The alliance whose nectar is bottom-most gets the Bottom NECTAR Bonus (null = nobody). */
    public Alliance bottomNectar() {
        for (Ball b : stack) {
            Alliance a = Alliance.ofNectar(b.kind);
            if (a != null) return a;
        }
        return null;
    }
}
