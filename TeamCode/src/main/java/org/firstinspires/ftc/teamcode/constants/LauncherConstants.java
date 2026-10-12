package org.firstinspires.ftc.teamcode.constants;

/**
 * Hardware names, speed target, tolerances, PID gains and feed timing for the Launcher (logged as "Shooter").
 *
 * Layer: constants. One file per subsystem, so every tunable number has one
 * home and no logic file has a number typed into it.
 * A zero here means "not measured yet". The Launcher refuses to reach READY
 * while TARGET_SPEED or ENTER_TOLERANCE is zero, so do not put a guess in.
 */
public final class LauncherConstants {

    public static final String LAUNCHER_NAME = "launcher";
    public static final boolean LAUNCHER_REVERSED = false;

    public static final double TARGET_SPEED = 0.0;
    public static final double ENTER_TOLERANCE = 0.0;
    public static final double LEAVE_TOLERANCE = 0.0;
    public static final double MAX_POWER = 1.0;

    public static final double KP = 0.0, KI = 0.0, KD = 0.0, KF = 0.0;
    public static final double I_LIMIT = 0.2;

    public static final String WINDMILL_NAME = "windmill";
    public static final boolean WINDMILL_REVERSED = false;
    public static final double WINDMILL_POWER = 1.0;
    public static final long QUARTER_TURN_MS = 0;
    public static final long MIN_FEED_GAP_MS = 0;

    private LauncherConstants() {}
}
