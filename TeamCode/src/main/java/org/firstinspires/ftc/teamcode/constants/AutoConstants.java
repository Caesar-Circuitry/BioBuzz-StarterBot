package org.firstinspires.ftc.teamcode.constants;

/**
 * Timeouts and counts for the autonomous routines.
 *
 * Layer: constants. One file per subsystem, so every tunable number has one
 * home and no logic file has a number typed into it.
 */
public final class AutoConstants {
    public static final  long PARK_TIMEOUT_MS = 8000;

    public static final int  PRELOAD_COUNT       = 4;      // UNMEASURED: how many pollen are preloaded
    public static final long READY_TIMEOUT_MS    = 3000;   // UNMEASURED: max wait per shot
    public static final long SHOOT_WINDOW_MS     = 15000;  // UNMEASURED: total shooting time, leaves room to park
    private AutoConstants(){}
}
