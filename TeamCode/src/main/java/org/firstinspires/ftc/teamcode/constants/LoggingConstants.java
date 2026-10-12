package org.firstinspires.ftc.teamcode.constants;

/**
 * Switches for PsiKit logging.
 *
 * Layer: constants. One file per subsystem, so every tunable number has one
 * home and no logic file has a number typed into it.
 * Keep LIVE_SERVER_ENABLED false at events: a live server costs loop time and
 * the log file on the robot is what we read afterwards.
 */
public final class LoggingConstants {
    public static final boolean LIVE_SERVER_ENABLED = false;
    public static final int LIVE_SERVER_PORT = 5800;

    private LoggingConstants() {}
}
