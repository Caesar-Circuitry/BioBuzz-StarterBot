package org.firstinspires.ftc.teamcode.sim.biobuzz;

public enum Alliance {
    RED, BLUE;

    static Alliance ofNectar(Ball.Kind kind) {
        if (kind == Ball.Kind.RED_NECTAR) return RED;
        if (kind == Ball.Kind.BLUE_NECTAR) return BLUE;
        return null;
    }
}
