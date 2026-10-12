package org.firstinspires.ftc.teamcode.constants;

/**
 * Numbers for the pollen drive assist (path planner footprint and the safety timeout).
 *
 * Layer: constants. One file per subsystem, so every tunable number has one
 * home and no logic file has a number typed into it.
 */
public final class AssistConstants {
    public static final double  INTAKE_WIDTH = 0.0;   // UNMEASURED: half the physical intake width. 0 = stop on target
    public static final double  ROBOT_WIDTH  = 16.0;  // UNMEASURED: footprint along the forward axis
    public static final double  ROBOT_HEIGHT = 16.0;  // UNMEASURED: footprint across the forward axis
    public static final double  CLEARANCE    = 4.0;   // Weaver's default
    public static final boolean SMOOTHING    = true;

    // Safety
    public static final long MAX_ASSIST_MS = 4000;    // UNMEASURED: give up after this long

    private AssistConstants() {}
}
