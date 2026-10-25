package org.firstinspires.ftc.teamcode.sim.biobuzz;

import java.util.ArrayList;
import java.util.List;

/**
 * A bi-stable see-saw with two cells that really rotates.
 *
 * <pre>
 *   side -1 cell            side +1 cell
 *        [   ]=====(pivot)=====[   ]        angle > 0 raises side +1
 * </pre>
 *
 * At rest the hive is LATCHED at +restAngle or -restAngle; the raised cell
 * faces up and catches balls. When the weight in it reaches the threshold the
 * latch lets go (TIPPING): the hive swings toward the other rest angle, the
 * balls ride along in the cell and fall out once it passes level. When it
 * reaches the other side it hits the damper (the moment the manual counts as a
 * TIP), bounces a little (SETTLING) and latches again with the other cell up.
 *
 * Geometry is described in the hive's own frame: "along" = the tip direction
 * (u), "across" = the pivot axis (p), "up" = vertical before rotation.
 */
public class Hive {
    public enum Phase { LATCHED, TIPPING, SETTLING }

    public final Alliance alliance;
    /** Pivot position, top view. */
    public final double x, y;
    /** Unit vector the hive tips along. */
    public final double ux, uy;
    /** Unit vector along the pivot axis. */
    public final double px, py;

    /** Current angle in radians. Positive raises side +1. */
    public double angle;
    public double angularVelocity;
    public Phase phase = Phase.LATCHED;
    public int tips = 0;

    /** Balls riding in the raised (catching) cell. */
    public final List<Ball> upCell = new ArrayList<>();
    /** Which side's cell holds upCell. */
    private int contentsSide;
    private int targetSide;
    /** Side of the alliance area along the pivot axis: where the first balls line up. */
    private final int acrossTowardAlliance;

    private final BioBuzzConfig c;

    Hive(Alliance alliance, double x, double y, double ux, double uy, int upSide,
         double allianceAreaDirX, double allianceAreaDirY, BioBuzzConfig c) {
        this.alliance = alliance;
        this.x = x;
        this.y = y;
        this.ux = ux;
        this.uy = uy;
        this.px = -uy;
        this.py = ux;
        this.c = c;
        this.angle = upSide * restAngle();
        this.contentsSide = upSide;
        double d = allianceAreaDirX * px + allianceAreaDirY * py;
        this.acrossTowardAlliance = d < 0 ? -1 : 1;
    }

    // ---------------------------------------------------------------- state

    public double restAngle() { return Math.toRadians(c.hiveRestAngleDeg); }

    public double pivotHeight() { return c.pivotHeight; }

    /** The side whose cell is raised right now (+1 or -1). */
    public int upSide() { return angle >= 0 ? 1 : -1; }

    public boolean canCatch() { return phase != Phase.TIPPING && upSide() == contentsSide; }

    public double upCellWeight() {
        double w = 0;
        for (Ball b : upCell) w += b.weight;
        return w;
    }

    // ---------------------------------------------------------------- geometry

    /** Local (along, across, up) -> world {x, y, z} at the current angle. */
    public double[] toWorld(double along, double across, double up) {
        double cos = Math.cos(angle), sin = Math.sin(angle);
        double a = along * cos - up * sin;
        double h = along * sin + up * cos;
        return new double[]{x + ux * a + px * across, y + uy * a + py * across, c.pivotHeight + h};
    }

    /** World -> local {along, across, up} at the current angle. */
    double[] toLocal(double wx, double wy, double wz) {
        double dx = wx - x, dy = wy - y;
        double a = dx * ux + dy * uy;
        double across = dx * px + dy * py;
        double h = wz - c.pivotHeight;
        double cos = Math.cos(angle), sin = Math.sin(angle);
        return new double[]{a * cos + h * sin, across, -a * sin + h * cos};
    }

    public double cellCenterAlong(int side) { return side * c.cellCenterAlong; }

    /** World position of a cell's centre. */
    public double[] cellCenter(int side) {
        return toWorld(cellCenterAlong(side), 0, c.cellCenterAbovePivot);
    }

    /** The 8 corners of a cell box in world coordinates (bottom 0-3, top 4-7). */
    public double[][] cellCorners(int side) {
        double ca = cellCenterAlong(side);
        double hd = c.cellDepth / 2, hw = c.cellWidth / 2, hh = c.cellHeight / 2;
        double ch = c.cellCenterAbovePivot;
        double[][] local = {
                {ca - hd, -hw, ch - hh}, {ca + hd, -hw, ch - hh}, {ca + hd, hw, ch - hh}, {ca - hd, hw, ch - hh},
                {ca - hd, -hw, ch + hh}, {ca + hd, -hw, ch + hh}, {ca + hd, hw, ch + hh}, {ca - hd, hw, ch + hh},
        };
        double[][] out = new double[8][];
        for (int i = 0; i < 8; i++) out[i] = toWorld(local[i][0], local[i][1], local[i][2]);
        return out;
    }

    /** Velocity of a world point because the hive is turning. */
    double[] surfaceVelocity(double wx, double wy, double wz) {
        double[] l = toLocal(wx, wy, wz);
        double cos = Math.cos(angle), sin = Math.sin(angle);
        // d/dt of (a cos - h sin, a sin + h cos) = omega * (-a sin - h cos, a cos - h sin)
        double dA = angularVelocity * (-l[0] * sin - l[2] * cos);
        double dH = angularVelocity * (l[0] * cos - l[2] * sin);
        return new double[]{ux * dA, uy * dA, dH};
    }

    // ---------------------------------------------------------------- balls

    /** Catch the ball if it is falling into the raised cell. */
    boolean tryCatch(Ball b) {
        if (!canCatch()) return false;
        double[] l = toLocal(b.x, b.y, b.z);
        double floor = c.cellCenterAbovePivot - c.cellHeight / 2 + c.cellFloorThickness;
        double out = contentsSide * l[0];                // distance from the pivot toward this cell
        if (out < c.cellInsideAlongMin || out > c.cellInsideAlongMax) return false;
        if (Math.abs(l[1]) > c.cellInsideWidth / 2) return false;
        if (l[2] < floor || l[2] > c.cellRimAbovePivot) return false;
        // velocity along the cell's own "up": must be moving down into it
        double vAlong = b.vx * ux + b.vy * uy;
        double vUpLocal = -vAlong * Math.sin(angle) + b.vz * Math.cos(angle);
        if (vUpLocal >= 0) return false;
        b.state = Ball.State.IN_CELL;
        upCell.add(b);
        arrangeCell();
        return true;
    }

    /** Push a free ball out of either cell's solid box. Returns true if it hit. */
    boolean collide(Ball b, double restitution) {
        boolean hit = false;
        for (int side = -1; side <= 1; side += 2) {
            double[] l = toLocal(b.x, b.y, b.z);
            double ca = cellCenterAlong(side);
            double oa = l[0] - ca, oc = l[1], oh = l[2] - c.cellCenterAbovePivot;
            double pa = c.cellDepth / 2 + b.radius - Math.abs(oa);
            double pc = c.cellWidth / 2 + b.radius - Math.abs(oc);
            double ph = c.cellHeight / 2 + b.radius - Math.abs(oh);
            if (pa <= 0 || pc <= 0 || ph <= 0) continue;

            boolean openCell = side == contentsSide && canCatch();
            boolean overOpening = Math.abs(oa) < c.cellDepth / 2 && Math.abs(oc) < c.cellWidth / 2;
            if (openCell && overOpening && oh > -c.cellHeight / 2) continue;   // going in through the top

            // push out through the face with the least overlap
            double na = 0, nc = 0, nh = 0, pen;
            if (pa <= pc && pa <= ph) { na = Math.signum(oa); pen = pa; }
            else if (pc <= ph) { nc = Math.signum(oc); pen = pc; }
            else { nh = Math.signum(oh); pen = ph; }
            if (na == 0 && nc == 0 && nh == 0) { nh = 1; pen = ph; }

            double cos = Math.cos(angle), sin = Math.sin(angle);
            double wa = na * cos - nh * sin, wh = na * sin + nh * cos;
            double nx = ux * wa + px * nc, ny = uy * wa + py * nc, nz = wh;

            b.x += nx * pen;
            b.y += ny * pen;
            b.z += nz * pen;

            double[] sv = surfaceVelocity(b.x, b.y, b.z);
            double rvx = b.vx - sv[0], rvy = b.vy - sv[1], rvz = b.vz - sv[2];
            double vn = rvx * nx + rvy * ny + rvz * nz;
            if (vn < 0) {
                b.vx -= (1 + restitution) * vn * nx;
                b.vy -= (1 + restitution) * vn * ny;
                b.vz -= (1 + restitution) * vn * nz;
            }
            hit = true;
        }
        return hit;
    }

    /**
     * Lay the cell's balls out in the cell's own frame so they move with it.
     * Matches the official field model: the first row sits on the cell floor,
     * a little toward the pivot from the cell centre, starting at the end
     * nearest the alliance area. Further rows go either side of it, then the
     * next layer stacks on top.
     */
    void arrangeCell() {
        double ca = cellCenterAlong(contentsSide);
        double spacing = c.nectarDiameter + 0.02;
        double innerWidth = c.cellWidth - 2 * c.cellWallThickness;
        double innerDepth = c.cellDepth - 2 * c.cellWallThickness;
        int perRow = Math.max(1, (int) Math.floor(innerWidth / spacing));
        int rowsPerLayer = Math.max(1, (int) Math.floor(innerDepth / spacing));
        double firstRow = ca - contentsSide * c.cellFirstRowTowardPivot;
        double floor = c.cellCenterAbovePivot - c.cellHeight / 2 + c.cellFloorThickness;
        double minAlong = ca - innerDepth / 2 + c.nectarDiameter / 2, maxAlong = ca + innerDepth / 2 - c.nectarDiameter / 2;
        for (int i = 0; i < upCell.size(); i++) {
            Ball b = upCell.get(i);
            int row = i / perRow, col = i % perRow;
            int layer = row / rowsPerLayer, rowInLayer = row % rowsPerLayer;
            // rows: 0, then +1, -1, +2, ... away from the first row
            int k = (rowInLayer + 1) / 2 * (rowInLayer % 2 == 1 ? 1 : -1);
            double along = Math.max(minAlong, Math.min(maxAlong, firstRow + k * spacing));
            double across = acrossTowardAlliance * (innerWidth / 2 - c.nectarDiameter / 2 - col * spacing);
            double up = floor + b.radius + layer * spacing;
            double[] w = toWorld(along, across, up);
            b.place(w[0], w[1], w[2]);
        }
    }

    // ---------------------------------------------------------------- motion

    /** What happened during one step. */
    static class StepResult {
        final List<Ball> dumped = new ArrayList<>();
        boolean released = false;
        boolean tipped = false;
    }

    StepResult step(double dt) {
        StepResult r = new StepResult();
        double rest = restAngle();

        if (phase == Phase.LATCHED) {
            angularVelocity = 0;
            angle = upSide() * rest;
            if (upCellWeight() >= c.tipThreshold) {
                phase = Phase.TIPPING;
                targetSide = -upSide();
                r.released = true;
            } else {
                return r;
            }
        }

        // swing toward the target side: latch/spring push + the weight of whatever is still aboard
        double accel = c.hiveSnapAccel + c.hiveWeightAccel * upCellWeight();
        angularVelocity += targetSide * accel * dt;
        angularVelocity *= Math.max(0, 1 - c.hiveDamping * dt);
        angle += angularVelocity * dt;

        // contents fall out once their cell drops past level
        if (!upCell.isEmpty() && contentsSide * angle < -Math.toRadians(c.hiveDumpAngleDeg)) {
            for (Ball b : upCell) {
                double[] sv = surfaceVelocity(b.x, b.y, b.z);
                b.state = Ball.State.FREE;
                b.vx = sv[0];
                b.vy = sv[1];
                b.vz = sv[2];
                r.dumped.add(b);
            }
            upCell.clear();
        }
        if (!upCell.isEmpty()) arrangeCell();

        // damper at the end stop
        if (targetSide * angle >= rest) {
            angle = targetSide * rest;
            if (phase == Phase.TIPPING) {
                phase = Phase.SETTLING;
                tips++;
                r.tipped = true;
                contentsSide = targetSide;   // the newly raised cell now catches
            }
            if (targetSide * angularVelocity > 0) angularVelocity = -angularVelocity * c.hiveDamperRestitution;
            if (Math.abs(angularVelocity) < 0.3) {
                angularVelocity = 0;
                phase = Phase.LATCHED;
            }
        }
        return r;
    }
}
