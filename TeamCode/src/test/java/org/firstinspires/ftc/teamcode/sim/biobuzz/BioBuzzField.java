package org.firstinspires.ftc.teamcode.sim.biobuzz;

import org.firstinspires.ftc.teamcode.sim.Util.SimIntakeConfig;
import org.firstinspires.ftc.teamcode.sim.Util.SimLauncherConfig;

import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Deque;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

/**
 * The whole BioBuzz game world: every pollen and nectar, both hives, the four
 * flowers, and the balls going into, sitting in and coming out of the robot.
 *
 * Pure Java on purpose: it knows nothing about FTCodeSim, the FTC SDK or
 * logging, so it can be tested on its own. {@link BioBuzzFieldSim} feeds it the
 * robot's pose and intake power every physics tick, and calls {@link #launch}
 * when the SimulatedLauncher's feeder pushes a ball into the flywheel.
 */
public class BioBuzzField {
    public final BioBuzzConfig config;
    /** Pickup zone, capacity and outtake: same object as the SimulatedIntake's config. */
    public final SimIntakeConfig intake;
    /** Where and at what angle balls leave: same object as the SimulatedLauncher's config. */
    public final SimLauncherConfig launcher;

    /** Every piece. */
    public final List<Ball> balls = new ArrayList<>();
    /** The same pieces split by type (each is one list in AdvantageScope). */
    public final List<Ball> pollen = new ArrayList<>();
    public final List<Ball> redNectar = new ArrayList<>();
    public final List<Ball> blueNectar = new ArrayList<>();
    public final Map<Alliance, Hive> hives = new EnumMap<>(Alliance.class);
    public final List<Flower> flowers = new ArrayList<>();
    /** Balls inside the robot, first in = first launched. */
    public final Deque<Ball> robotStorage = new ArrayDeque<>();
    private final Map<Alliance, Deque<Ball>> stagedNectar = new EnumMap<>(Alliance.class);

    private double intakeTimer = 0;
    private double time = 0;

    private RobotInput robot = new RobotInput();
    private Random random;
    private final List<String> events = new ArrayList<>();

    public BioBuzzField(BioBuzzConfig config, SimIntakeConfig intake, SimLauncherConfig launcher) {
        this.config = config;
        this.intake = intake;
        this.launcher = launcher;
        reset();
    }

    /** With the default (StarterBot) intake and launcher settings. */
    public BioBuzzField(BioBuzzConfig config) {
        this(config, new SimIntakeConfig(), new SimLauncherConfig());
    }

    // =====================================================================
    // Setup
    // =====================================================================

    /** Put every piece back where the manual says it starts a match. */
    public void reset() {
        BioBuzzConfig c = config;
        balls.clear();
        pollen.clear();
        redNectar.clear();
        blueNectar.clear();
        hives.clear();
        flowers.clear();
        robotStorage.clear();
        stagedNectar.put(Alliance.RED, new ArrayDeque<>());
        stagedNectar.put(Alliance.BLUE, new ArrayDeque<>());
        intakeTimer = 0;
        time = 0;
        random = new Random(c.randomSeed);
        events.clear();

        robot = new RobotInput();
        robot.x = c.startX;
        robot.y = c.startY;
        robot.heading = c.startHeading;

        // Hives
        double half = c.hiveSpacing / 2;
        double lowSign = c.redHiveOnLowSide ? -1 : 1;
        double ux = c.hivesAlongX ? 0 : 1, uy = c.hivesAlongX ? 1 : 0;
        double ox = c.hivesAlongX ? half : 0, oy = c.hivesAlongX ? 0 : half;
        double rx = c.hiveStructureX + lowSign * ox - ux * c.hivePivotShift;
        double ry = c.hiveStructureY + lowSign * oy - uy * c.hivePivotShift;
        double bx = c.hiveStructureX - lowSign * ox + ux * c.hivePivotShift;
        double by = c.hiveStructureY - lowSign * oy + uy * c.hivePivotShift;
        hives.put(Alliance.RED, new Hive(Alliance.RED, rx, ry, ux, uy, c.redInitialUpSide,
                c.redNectarSpawn[0] - rx, c.redNectarSpawn[1] - ry, c));
        hives.put(Alliance.BLUE, new Hive(Alliance.BLUE, bx, by, ux, uy, c.blueInitialUpSide,
                c.blueNectarSpawn[0] - bx, c.blueNectarSpawn[1] - by, c));

        // Nectar: 3 in each up cell, 5 staged per alliance
        for (Alliance a : Alliance.values()) {
            Ball.Kind kind = a == Alliance.RED ? Ball.Kind.RED_NECTAR : Ball.Kind.BLUE_NECTAR;
            Hive hive = hives.get(a);
            for (int i = 0; i < c.nectarPerCellStart; i++) {
                Ball b = newBall(kind);
                b.state = Ball.State.IN_CELL;
                hive.upCell.add(b);
            }
            hive.arrangeCell();
            for (int i = 0; i < c.nectarStagedPerAlliance; i++) {
                Ball b = newBall(kind);
                b.state = Ball.State.STAGED;
                b.place(-50, -50, -100);
                stagedNectar.get(a).add(b);
            }
        }

        // Flowers: 4 pollen each
        for (int i = 0; i < c.flowerPositions.length; i++) {
            Flower f = new Flower(i, c.flowerPositions[i][0], c.flowerPositions[i][1], c);
            for (int k = 0; k < c.pollenPerFlowerStart; k++) {
                Ball b = newBall(Ball.Kind.POLLEN);
                b.state = Ball.State.IN_FLOWER;
                f.stack.add(b);
            }
            f.arrange();
            flowers.add(f);
        }

        // Gardens: a line of pollen along the wall, starting in the corner
        layGarden(c.redGarden);
        layGarden(c.blueGarden);

        // Robot preloads
        for (int i = 0; i < c.preloadPollen; i++) {
            Ball b = newBall(Ball.Kind.POLLEN);
            b.state = Ball.State.IN_ROBOT;
            robotStorage.add(b);
        }
        arrangeStorage();
    }

    private void layGarden(double[] rect) {
        double r = config.pollenDiameter / 2;
        boolean startAtLowX = rect[0] <= 0.01;
        double y = rect[1] <= 0.01 ? rect[1] + r : rect[3] - r;
        for (int i = 0; i < config.pollenPerGardenStart; i++) {
            Ball b = newBall(Ball.Kind.POLLEN);
            double x = startAtLowX ? rect[0] + r + i * (2 * r + 0.1) : rect[2] - r - i * (2 * r + 0.1);
            b.place(x, y, r);
        }
    }

    private Ball newBall(Ball.Kind kind) {
        String prefix = kind == Ball.Kind.POLLEN ? "Pollen" : kind == Ball.Kind.RED_NECTAR ? "RedNectar" : "BlueNectar";
        String name = prefix + listFor(kind).size();
        double d = kind == Ball.Kind.POLLEN ? config.pollenDiameter : config.nectarDiameter;
        double w = kind == Ball.Kind.POLLEN ? 1.0 : config.nectarWeight;
        Ball b = new Ball(balls.size(), kind, name, d / 2, w);
        balls.add(b);
        listFor(kind).add(b);
        return b;
    }

    public List<Ball> listFor(Ball.Kind kind) {
        switch (kind) {
            case POLLEN: return pollen;
            case RED_NECTAR: return redNectar;
            default: return blueNectar;
        }
    }

    // =====================================================================
    // Simulation step
    // =====================================================================

    /** Advance the world by dt seconds with the robot in the given state. */
    public void step(double dt, RobotInput input) {
        if (dt <= 0) return;
        dt = Math.min(dt, 0.05);                       // ignore huge gaps (e.g. first tick, debugger pause)
        robot = input;

        int n = (int) Math.ceil(dt / config.maxSubstep);
        double h = dt / n;
        for (int i = 0; i < n; i++) {
            time += h;
            for (Hive hive : hives.values()) stepHive(hive, h);
            updateIntake(h);
            for (Ball b : balls) {
                if (b.state == Ball.State.FREE) stepBall(b, h);
            }
            collideBalls();
        }
        arrangeStorage();
    }

    // ---------------- launcher ----------------

    /**
     * The feeder pushed one ball into the flywheel: send the next ball out of the
     * robot at the given exit speed (in/s), from the launcher's exit point and angle.
     * Returns false if the robot was empty.
     */
    public boolean launch(double speed) {
        Ball b = robotStorage.pollFirst();
        if (b == null) return false;

        double yaw = robot.heading + Math.toRadians(launcher.launchYawDeg);
        double pitch = Math.toRadians(launcher.launchAngleDeg);

        double[] p = robotToField(launcher.exitX, launcher.exitY);
        double[] v = pointVelocity(p[0], p[1]);
        b.state = Ball.State.FREE;
        b.place(p[0], p[1], launcher.exitHeight);
        b.vx = speed * Math.cos(pitch) * Math.cos(yaw) + v[0];
        b.vy = speed * Math.cos(pitch) * Math.sin(yaw) + v[1];
        b.vz = speed * Math.sin(pitch);

        event(String.format("Launched %s at %.0f in/s", b.name, speed));
        return true;
    }

    // ---------------- intake ----------------

    private void updateIntake(double h) {
        intakeTimer -= h;
        double power = robot.intakePower;              // already signed: + = pulling in
        if (Math.abs(power) < intake.minPower || intakeTimer > 0) return;

        if (power > 0) {
            if (robotStorage.size() >= intake.storageCapacity) return;
            // pick a ball off the floor
            for (Ball b : balls) {
                if (b.state == Ball.State.FREE && b.z - b.radius < intake.maxBallHeight && inIntakeZone(b.x, b.y, b.radius)) {
                    b.state = Ball.State.IN_ROBOT;
                    robotStorage.addLast(b);
                    intakeTimer = intake.cooldown;
                    event("Intook " + b.name);
                    return;
                }
            }
            // pull the bottom piece out of a flower's retrieval opening
            for (Flower f : flowers) {
                if (!f.stack.isEmpty() && inIntakeZone(f.x, f.y, config.flowerBodyRadius)) {
                    Ball b = f.stack.remove(0);
                    b.state = Ball.State.IN_ROBOT;
                    robotStorage.addLast(b);
                    f.arrange();
                    intakeTimer = intake.cooldown;
                    event("Pulled " + b.name + " out of flower " + f.index);
                    return;
                }
            }
        } else {
            // reverse: spit one ball out of the front
            Ball b = robotStorage.pollLast();
            if (b == null) return;
            double[] p = robotToField(config.robotCenterOffsetX + config.robotLength / 2 + b.radius + 0.5, 0);
            double[] v = pointVelocity(p[0], p[1]);
            b.state = Ball.State.FREE;
            b.place(p[0], p[1], b.radius + 0.5);
            b.vx = Math.cos(robot.heading) * intake.outtakeSpeed + v[0];
            b.vy = Math.sin(robot.heading) * intake.outtakeSpeed + v[1];
            intakeTimer = intake.cooldown * 2;
            event("Outtook " + b.name);
        }
    }

    private boolean inIntakeZone(double x, double y, double r) {
        double[] l = fieldToRobot(x, y);
        double front = config.robotCenterOffsetX + config.robotLength / 2;
        return l[0] + r >= front - 0.5 && l[0] - r <= front + intake.zoneDepth
                && Math.abs(l[1]) <= intake.zoneWidth / 2;
    }

    // ---------------- one free ball ----------------

    private void stepBall(Ball b, double h) {
        BioBuzzConfig c = config;
        double prevZ = b.z;

        boolean resting = b.z <= b.radius + 1e-3 && Math.abs(b.vz) < 1e-3;
        if (!resting) b.vz -= c.gravity * h;
        double drag = Math.max(0, 1 - c.airDrag * h);
        b.vx *= drag;
        b.vy *= drag;
        b.vz *= drag;

        b.x += b.vx * h;
        b.y += b.vy * h;
        b.z += b.vz * h;

        // floor
        if (b.z < b.radius) {
            b.z = b.radius;
            if (b.vz < 0) b.vz = -b.vz * c.floorRestitution;
            if (Math.abs(b.vz) < 8) b.vz = 0;
        }
        if (b.z <= b.radius + 1e-3 && b.vz == 0) {
            double speed = Math.hypot(b.vx, b.vy);
            double dec = c.rollingDecel * h;
            if (speed <= dec) {
                b.vx = b.vy = 0;
            } else {
                b.vx -= b.vx / speed * dec;
                b.vy -= b.vy / speed * dec;
            }
        }

        // perimeter walls
        double max = c.fieldSize;
        if (b.x < b.radius) { b.x = b.radius; if (b.vx < 0) b.vx = -b.vx * c.wallRestitution; }
        if (b.x > max - b.radius) { b.x = max - b.radius; if (b.vx > 0) b.vx = -b.vx * c.wallRestitution; }
        if (b.y < b.radius) { b.y = b.radius; if (b.vy < 0) b.vy = -b.vy * c.wallRestitution; }
        if (b.y > max - b.radius) { b.y = max - b.radius; if (b.vy > 0) b.vy = -b.vy * c.wallRestitution; }

        // flowers: drop in the top, otherwise bounce off the tube
        for (Flower f : flowers) {
            double dx = b.x - f.x, dy = b.y - f.y;
            double d = Math.hypot(dx, dy);
            boolean overOpening = d <= c.flowerOpeningDiameter / 2;
            boolean crossedTop = prevZ >= c.flowerTopHeight && b.z < c.flowerTopHeight && b.vz < 0;
            if (overOpening && crossedTop) {
                b.state = Ball.State.IN_FLOWER;
                f.stack.add(b);
                f.arrange();
                event(b.name + " dropped into flower " + f.index);
                return;
            }
            if (overOpening && b.z >= c.flowerTopHeight) continue;   // lined up with the hole: no rim hit
            double minD = c.flowerBodyRadius + b.radius;
            if (b.z - b.radius < c.flowerTopHeight && d < minD) {
                if (b.z > c.flowerTopHeight - 0.5 && prevZ >= c.flowerTopHeight) {
                    // landed on the rim: bounce up
                    b.z = c.flowerTopHeight + b.radius;
                    b.vz = Math.abs(b.vz) * c.wallRestitution;
                } else {
                    double nx = d < 1e-6 ? 1 : dx / d, ny = d < 1e-6 ? 0 : dy / d;
                    b.x = f.x + nx * minD;
                    b.y = f.y + ny * minD;
                    double vn = b.vx * nx + b.vy * ny;
                    if (vn < 0) {
                        b.vx -= (1 + c.wallRestitution) * vn * nx;
                        b.vy -= (1 + c.wallRestitution) * vn * ny;
                    }
                }
            }
        }

        // hive: falling into the raised cell catches the ball, otherwise bounce off the cells
        for (Hive hive : hives.values()) {
            if (hive.tryCatch(b)) {
                event(String.format("%s landed in the %s hive (%.1f / %.1f)", b.name, hive.alliance,
                        hive.upCellWeight(), c.tipThreshold));
                return;
            }
            hive.collide(b, c.wallRestitution);
        }

        collideRobot(b);
    }

    private void stepHive(Hive hive, double h) {
        Hive.StepResult r = hive.step(h);
        if (r.released) event(hive.alliance + " hive is tipping...");
        if (!r.dumped.isEmpty()) event(r.dumped.size() + " pieces fell out of the " + hive.alliance + " hive");
        if (r.tipped) {
            event(hive.alliance + " HIVE TIPPED! (" + hive.tips + " tips)");
            Ball nectar = stagedNectar.get(hive.alliance).pollFirst();
            if (nectar != null) {
                double[] sp = hive.alliance == Alliance.RED ? config.redNectarSpawn : config.blueNectarSpawn;
                nectar.state = Ball.State.FREE;
                nectar.place(sp[0] + (random.nextDouble() - 0.5) * 4, sp[1] + (random.nextDouble() - 0.5) * 4, nectar.radius + 6);
                event(nectar.name + " released into the " + hive.alliance + " alliance area");
            }
        }
    }

    /** Ball against the robot's box. The robot is treated as infinitely heavy. */
    private void collideRobot(Ball b) {
        BioBuzzConfig c = config;
        if (b.z - b.radius > c.robotHeight) return;

        double[] l = fieldToRobot(b.x, b.y);
        l[0] -= c.robotCenterOffsetX;                 // box centre is ahead of the drivetrain centre
        double hx = c.robotLength / 2, hy = c.robotWidth / 2;
        double cx = clamp(l[0], -hx, hx), cy = clamp(l[1], -hy, hy);
        double dx = l[0] - cx, dy = l[1] - cy;
        double d = Math.hypot(dx, dy);

        double nxL, nyL, pen;
        if (d > 1e-9) {
            if (d >= b.radius) return;
            nxL = dx / d;
            nyL = dy / d;
            pen = b.radius - d;
        } else {
            // centre is inside the robot: push out the nearest side
            double ex = hx - Math.abs(l[0]), ey = hy - Math.abs(l[1]);
            if (ex < ey) { nxL = Math.signum(l[0]); nyL = 0; pen = ex + b.radius; }
            else { nxL = 0; nyL = Math.signum(l[1]); pen = ey + b.radius; }
            if (nxL == 0 && nyL == 0) nxL = 1;
        }

        double cos = Math.cos(robot.heading), sin = Math.sin(robot.heading);
        double nx = nxL * cos - nyL * sin, ny = nxL * sin + nyL * cos;
        b.x += nx * pen;
        b.y += ny * pen;

        double[] pv = pointVelocity(b.x, b.y);
        double rvx = b.vx - pv[0], rvy = b.vy - pv[1];
        double vn = rvx * nx + rvy * ny;
        if (vn < 0) {
            b.vx -= (1 + c.robotRestitution) * vn * nx;
            b.vy -= (1 + c.robotRestitution) * vn * ny;
        }
    }

    private void collideBalls() {
        double e = config.ballRestitution;
        for (int i = 0; i < balls.size(); i++) {
            Ball a = balls.get(i);
            if (a.state != Ball.State.FREE) continue;
            for (int j = i + 1; j < balls.size(); j++) {
                Ball b = balls.get(j);
                if (b.state != Ball.State.FREE) continue;
                double dx = b.x - a.x, dy = b.y - a.y, dz = b.z - a.z;
                double minD = a.radius + b.radius;
                double d2 = dx * dx + dy * dy + dz * dz;
                if (d2 >= minD * minD) continue;
                double d = Math.sqrt(d2);
                double nx, ny, nz;
                if (d < 1e-9) { nx = 1; ny = 0; nz = 0; d = 0; }
                else { nx = dx / d; ny = dy / d; nz = dz / d; }

                double ma = a.weight, mb = b.weight, total = ma + mb;
                double pen = minD - d;
                a.x -= nx * pen * mb / total; a.y -= ny * pen * mb / total; a.z -= nz * pen * mb / total;
                b.x += nx * pen * ma / total; b.y += ny * pen * ma / total; b.z += nz * pen * ma / total;
                if (a.z < a.radius) a.z = a.radius;
                if (b.z < b.radius) b.z = b.radius;

                double vn = (b.vx - a.vx) * nx + (b.vy - a.vy) * ny + (b.vz - a.vz) * nz;
                if (vn >= 0) continue;
                double jImp = -(1 + e) * vn / (1 / ma + 1 / mb);
                a.vx -= jImp / ma * nx; a.vy -= jImp / ma * ny; a.vz -= jImp / ma * nz;
                b.vx += jImp / mb * nx; b.vy += jImp / mb * ny; b.vz += jImp / mb * nz;
            }
        }
    }

    // ---------------- robot storage (for display) ----------------

    private void arrangeStorage() {
        int i = 0;
        for (Ball b : robotStorage) {
            b.place(robot.x, robot.y, config.robotHeight + b.radius + i * 2 * b.radius);
            i++;
        }
    }

    // =====================================================================
    // Read-outs
    // =====================================================================

    public Score score(Alliance a) {
        Score s = new Score();
        Hive hive = hives.get(a);
        s.tips = hive.tips;
        s.piecesInCells = hive.upCell.size();
        for (Flower f : flowers) {
            if (f.owner() == a) {
                s.flowersOwned++;
                s.piecesInOwnedFlowers += f.stack.size();
            }
            if (f.bottomNectar() == a) s.bottomNectarBonuses++;
        }
        double[] g = a == Alliance.RED ? config.redGarden : config.blueGarden;
        for (Ball b : balls) {
            if (b.onFloor() && b.x >= g[0] && b.x <= g[2] && b.y >= g[1] && b.y <= g[3]) s.piecesInGarden++;
        }
        return s;
    }

    // =====================================================================
    // Layout check
    // =====================================================================

    /**
     * Checks the starting layout against the manual and against itself.
     * Returns a list of problems; empty means everything checks out.
     * Call right after reset() (BioBuzzFieldSim does this and prints the result).
     */
    public List<String> validate() {
        List<String> problems = new ArrayList<>();
        BioBuzzConfig c = config;
        double eps = 0.05;

        // ---- counts (MANUAL 9.x / 10.3.1) ----
        int robotsOnField = 1;
        expect(problems, pollen.size() == c.pollenPerFlowerStart * c.flowerPositions.length
                        + 2 * c.pollenPerGardenStart + c.preloadPollen * robotsOnField,
                "pollen count " + pollen.size());
        expect(problems, redNectar.size() == 8 && blueNectar.size() == 8,
                "nectar count red " + redNectar.size() + " blue " + blueNectar.size() + " (manual: 8 + 8)");
        expect(problems, c.flowerPositions.length == 4, "manual has 4 flowers, config has " + c.flowerPositions.length);
        expect(problems, Math.abs(c.pollenDiameter - 2.8) < eps && Math.abs(c.nectarDiameter - 3.6) < eps,
                "piece diameters differ from the manual (2.8 / 3.6 in)");
        expect(problems, Math.abs(c.pivotHeight - 43.95) < 1.0, "pivot height is more than 1 in from the manual (43.95 in)");
        expect(problems, c.cellWidth - 2 * c.cellWallThickness >= 19.5, "cell opening is narrower than the manual's ~20 in");
        expect(problems, Math.abs(c.flowerTopHeight - 21.5) < eps, "flower top differs from the manual (21.5 in)");

        // ---- every free piece is on the field, on the floor, not overlapping anything ----
        for (Ball b : balls) {
            if (b.state != Ball.State.FREE) continue;
            expect(problems, b.x >= b.radius - eps && b.x <= c.fieldSize - b.radius + eps
                    && b.y >= b.radius - eps && b.y <= c.fieldSize - b.radius + eps, b.name + " is outside the walls");
            expect(problems, Math.abs(b.z - b.radius) < eps, b.name + " is not resting on the tiles");
            expect(problems, !insideRobotFootprint(b.x, b.y, b.radius), b.name + " starts inside the robot");
            expect(problems, !insideHiveFrame(b.x, b.y, b.radius), b.name + " starts under the hive frame");
        }
        for (int i = 0; i < balls.size(); i++) {
            Ball a = balls.get(i);
            if (a.state == Ball.State.STAGED || a.state == Ball.State.IN_ROBOT) continue;
            for (int j = i + 1; j < balls.size(); j++) {
                Ball b = balls.get(j);
                if (b.state == Ball.State.STAGED || b.state == Ball.State.IN_ROBOT) continue;
                double d = Math.sqrt(sq(a.x - b.x) + sq(a.y - b.y) + sq(a.z - b.z));
                expect(problems, d >= a.radius + b.radius - eps,
                        String.format("%s and %s overlap (%.2f in apart)", a.name, b.name, d));
            }
        }

        // ---- gardens (10.3.1: a line from the corner nearest the alliance area, touching the wall) ----
        for (Alliance al : Alliance.values()) {
            double[] g = al == Alliance.RED ? c.redGarden : c.blueGarden;
            expect(problems, g[0] >= -eps && g[1] >= -eps && g[2] <= c.fieldSize + eps && g[3] <= c.fieldSize + eps,
                    al + " garden is outside the field");
            boolean touchesWall = g[1] <= eps || g[3] >= c.fieldSize - eps;
            boolean inCorner = touchesWall && (g[0] <= eps || g[2] >= c.fieldSize - eps);
            expect(problems, inCorner, al + " garden is not in a corner against the audience/rear wall");
            double[] sp = al == Alliance.RED ? c.redNectarSpawn : c.blueNectarSpawn;
            boolean gardenOnAllianceSide = Math.abs((g[0] + g[2]) / 2 - sp[0]) < c.fieldSize / 2;
            expect(problems, gardenOnAllianceSide, al + " garden is not on the same side as its alliance area");
            expect(problems, score(al).piecesInGarden == c.pollenPerGardenStart,
                    al + " garden starts with " + score(al).piecesInGarden + " scored pieces, expected " + c.pollenPerGardenStart);
        }

        // ---- hives ----
        for (Hive h : hives.values()) {
            double[] pivotOff = toFrameLocal(h.x, h.y);
            expect(problems, Math.abs(pivotOff[0]) <= c.hiveFrameWidth / 2 && Math.abs(pivotOff[1]) <= c.hiveFrameDepth / 2,
                    h.alliance + " hive pivot is outside the frame");
            for (int side = -1; side <= 1; side += 2) {
                for (double[] corner : h.cellCorners(side)) {
                    expect(problems, corner[2] > c.robotHeight,
                            String.format("%s hive cell %d hangs down to %.1f in, into robots (robotHeight %.0f)", h.alliance, side, corner[2], c.robotHeight));
                    expect(problems, corner[0] > 0 && corner[0] < c.fieldSize && corner[1] > 0 && corner[1] < c.fieldSize,
                            h.alliance + " hive cell " + side + " pokes outside the field");
                }
            }
            expect(problems, h.upCell.size() == c.nectarPerCellStart, h.alliance + " up cell should start with "
                    + c.nectarPerCellStart + " nectar, has " + h.upCell.size());
            for (Ball b : h.upCell) {
                Ball.Kind want = h.alliance == Alliance.RED ? Ball.Kind.RED_NECTAR : Ball.Kind.BLUE_NECTAR;
                expect(problems, b.kind == want, b.name + " is in the wrong colour hive");
                double[] l = h.toLocal(b.x, b.y, b.z);
                double ca = h.cellCenterAlong(h.upSide());
                expect(problems, Math.abs(l[0] - ca) <= c.cellDepth / 2 - b.radius + eps
                                && Math.abs(l[1]) <= c.cellWidth / 2 - b.radius + eps
                                && Math.abs(l[2] - c.cellCenterAbovePivot) <= c.cellHeight / 2 - b.radius + eps,
                        b.name + " is not fully inside its cell");
            }
            expect(problems, h.upCellWeight() < c.tipThreshold,
                    h.alliance + " hive would tip immediately (starting weight " + h.upCellWeight() + " >= threshold)");
        }
        Hive red = hives.get(Alliance.RED), blue = hives.get(Alliance.BLUE);
        double hiveGap = Math.hypot(red.x - blue.x, red.y - blue.y);
        expect(problems, hiveGap >= c.cellWidth - eps, String.format("red and blue hive cells overlap (centres %.1f in apart, cells are %.0f wide)", hiveGap, c.cellWidth));

        // ---- flowers (9.x: mounted on the perimeter wall) ----
        for (Flower f : flowers) {
            double wallDist = Math.min(Math.min(f.x, c.fieldSize - f.x), Math.min(f.y, c.fieldSize - f.y));
            expect(problems, wallDist <= c.flowerBodyRadius + 0.5, "flower " + f.index + " is not against a wall ("
                    + String.format("%.1f", wallDist) + " in away)");
            expect(problems, !insideHiveFrame(f.x, f.y, c.flowerBodyRadius), "flower " + f.index + " is under the hive");
            double top = f.stack.isEmpty() ? 0 : f.stack.get(f.stack.size() - 1).z + f.stack.get(f.stack.size() - 1).radius;
            expect(problems, top <= c.flowerTopHeight + eps, "flower " + f.index + " is over-full at the start");
            expect(problems, c.flowerOpeningDiameter > c.nectarDiameter, "nectar would not fit in a flower");
        }
        for (int i = 0; i < flowers.size(); i++)
            for (int j = i + 1; j < flowers.size(); j++)
                expect(problems, Math.hypot(flowers.get(i).x - flowers.get(j).x, flowers.get(i).y - flowers.get(j).y) > 2 * c.flowerBodyRadius,
                        "flowers " + i + " and " + j + " overlap");

        // ---- robot start and nectar drop spots ----
        double[][] corners = robotCorners();
        for (double[] p : corners) {
            expect(problems, p[0] >= -eps && p[0] <= c.fieldSize + eps && p[1] >= -eps && p[1] <= c.fieldSize + eps,
                    "robot start pose puts the robot through a wall");
            expect(problems, !insideHiveFrame(p[0], p[1], 0), "robot starts under the hive frame");
        }
        for (Alliance al : Alliance.values()) {
            double[] sp = al == Alliance.RED ? c.redNectarSpawn : c.blueNectarSpawn;
            expect(problems, sp[0] > 0 && sp[0] < c.fieldSize && sp[1] > 0 && sp[1] < c.fieldSize, al + " nectar spawn is off the field");
            expect(problems, !insideHiveFrame(sp[0], sp[1], c.nectarDiameter / 2), al + " nectar spawn is under the hive");
        }
        expect(problems, robotStorage.size() == c.preloadPollen && c.preloadPollen <= intake.storageCapacity,
                "robot should start holding " + c.preloadPollen + " preloads");
        // ---- against the official AdvantageScope field model ----
        if (c.compareToFieldModel) problems.addAll(FieldReference.compare(this, 1.0));
        return problems;
    }

    private static void expect(List<String> problems, boolean ok, String message) {
        if (!ok) problems.add(message);
    }

    private static double sq(double v) { return v * v; }

    /** Offset from the hive frame centre, {across the frame width, along its depth}. */
    private double[] toFrameLocal(double x, double y) {
        double dx = x - config.hiveStructureX, dy = y - config.hiveStructureY;
        return config.hivesAlongX ? new double[]{dx, dy} : new double[]{dy, dx};
    }

    private boolean insideHiveFrame(double x, double y, double r) {
        double[] o = toFrameLocal(x, y);
        return Math.abs(o[0]) < config.hiveFrameWidth / 2 + r && Math.abs(o[1]) < config.hiveFrameDepth / 2 + r;
    }

    private boolean insideRobotFootprint(double x, double y, double r) {
        double[] l = fieldToRobot(x, y);
        l[0] -= config.robotCenterOffsetX;
        return Math.abs(l[0]) < config.robotLength / 2 + r && Math.abs(l[1]) < config.robotWidth / 2 + r;
    }

    private double[][] robotCorners() {
        double hl = config.robotLength / 2, hw = config.robotWidth / 2, o = config.robotCenterOffsetX;
        return new double[][]{robotToField(o + hl, hw), robotToField(o + hl, -hw), robotToField(o - hl, -hw), robotToField(o - hl, hw)};
    }

    public int stagedNectar(Alliance a) { return stagedNectar.get(a).size(); }

    public double time() { return time; }

    /** Messages since the last call (launches, pickups, tips...). */
    public List<String> drainEvents() {
        if (events.isEmpty()) return Collections.emptyList();
        List<String> out = new ArrayList<>(events);
        events.clear();
        return out;
    }

    private void event(String msg) {
        events.add(String.format("[%.2fs] %s", time, msg));
    }

    // ---------------- geometry helpers ----------------

    private double[] robotToField(double lx, double ly) {
        double cos = Math.cos(robot.heading), sin = Math.sin(robot.heading);
        return new double[]{robot.x + lx * cos - ly * sin, robot.y + lx * sin + ly * cos};
    }

    private double[] fieldToRobot(double x, double y) {
        double dx = x - robot.x, dy = y - robot.y;
        double cos = Math.cos(robot.heading), sin = Math.sin(robot.heading);
        return new double[]{dx * cos + dy * sin, -dx * sin + dy * cos};
    }

    /** Velocity of a point on the robot: v + omega x r. */
    private double[] pointVelocity(double x, double y) {
        double rx = x - robot.x, ry = y - robot.y;
        return new double[]{robot.vx - robot.omega * ry, robot.vy + robot.omega * rx};
    }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}
