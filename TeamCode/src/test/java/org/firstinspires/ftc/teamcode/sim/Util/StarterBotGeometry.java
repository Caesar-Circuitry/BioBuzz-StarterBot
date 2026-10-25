package org.firstinspires.ftc.teamcode.sim.Util;

import org.codeblooded.ftcodesim.ascope.boundaries.RobotGeometry;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimMecanumConfig;
import org.firstinspires.ftc.teamcode.sim.biobuzz.BioBuzzConfig;

/**
 * StarterBot dimensions, measured from AScope_Assets/Robot_StarterBot/model.glb.
 * One source of truth for the drivetrain sim, the field (collisions, intake zone)
 * and the launcher exit point.
 *
 * Robot frame: origin = drivetrain centre (centre of the four wheels, what a mecanum
 * turns around), x forward, y left, z up from the tiles. Inches.
 */
public final class StarterBotGeometry {
    private StarterBotGeometry() {}

    /** goBILDA 3625 mecanum wheels, 104 mm. */
    public static final double WHEEL_RADIUS = 2.047;
    /** Front wheel axle to rear wheel axle (wheel centres at x +5.20 / -5.20). */
    public static final double WHEELBASE = 10.39;
    /** Left wheel centre to right wheel centre (y +-8.15). */
    public static final double TRACK_WIDTH = 16.30;

    /** Frame: x from -8.03 (back) to +9.77 (front bumper), y +-8.90, top of the hood 12.69. */
    public static final double LENGTH = 17.80;
    public static final double WIDTH = 17.80;
    public static final double HEIGHT = 12.69;
    /** The frame's centre is this far ahead of the drivetrain centre. */
    public static final double FRAME_CENTER_AHEAD = 0.87;

    /** FTCodeSim drivetrain: wheel layout and the box it uses for wall collisions. */
    public static void applyTo(SimMecanumConfig config) {
        config.wheelRadius = WHEEL_RADIUS;
        config.wheelbase = WHEELBASE;
        config.trackWidth = TRACK_WIDTH;
        config.robotGeometry = new RobotGeometry(WIDTH, LENGTH, FRAME_CENTER_AHEAD, 0);
    }

    /** BioBuzz field: the box balls bounce off and the intake zone in front of it. */
    public static void applyTo(BioBuzzConfig config) {
        config.robotLength = LENGTH;
        config.robotWidth = WIDTH;
        config.robotHeight = HEIGHT;
        config.robotCenterOffsetX = FRAME_CENTER_AHEAD;
    }
}
