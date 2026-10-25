package org.firstinspires.ftc.teamcode.sim.biobuzz;

/** One pollen or nectar ball. Position and velocity are in field inches. */
public class Ball {
    public enum Kind {
        POLLEN, RED_NECTAR, BLUE_NECTAR;

        public boolean isNectar() { return this != POLLEN; }
    }

    public enum State {
        /** Moving under physics on the field. */
        FREE,
        /** Inside the robot, waiting to be launched. */
        IN_ROBOT,
        /** Sitting in a hive's upward-facing cell. */
        IN_CELL,
        /** Stacked inside a flower. */
        IN_FLOWER,
        /** Nectar waiting in an alliance area to be released by a tip. */
        STAGED
    }

    public final int id;
    public final Kind kind;
    public final String name;
    public final double radius;
    public final double weight;

    public double x, y, z;
    public double vx, vy, vz;
    public State state = State.FREE;

    Ball(int id, Kind kind, String name, double radius, double weight) {
        this.id = id;
        this.kind = kind;
        this.name = name;
        this.radius = radius;
        this.weight = weight;
    }

    void place(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
        vx = vy = vz = 0;
    }

    public boolean onFloor() {
        return state == State.FREE && z <= radius + 0.05 && Math.abs(vz) < 1;
    }

    @Override
    public String toString() {
        return String.format("%s[%s %.1f,%.1f,%.1f]", name, state, x, y, z);
    }
}
