package org.firstinspires.ftc.teamcode.StarterBot;

/**
 * Switches for PsiKit logging (same switches as the Cohen-Testing branch).
 *
 * Keep LIVE_SERVER_ENABLED false at events: a live server costs loop time and
 * the log file on the robot (/sdcard/FIRST/PsiKit/) is what we read afterwards.
 *
 * In the simulator the live server is always on, because that is how
 * AdvantageScope watches the sim, and an extra copy of every log is written to
 * SIM_LOG_FOLDER (TeamCode/sim-logs/) since the Mac has no /sdcard.
 */
public final class LoggingConstants {
    public static final boolean LIVE_SERVER_ENABLED = false;
    public static final int LIVE_SERVER_PORT = 5800;

    /** True when running in FTCodeSim (its classes are only on the test classpath, never on the robot). */
    public static final boolean IN_SIMULATOR = detectSimulator();
    public static final String SIM_LOG_FOLDER = "sim-logs/";

    private LoggingConstants() {}

    private static boolean detectSimulator() {
        try {
            Class.forName("org.codeblooded.ftcodesim.simulator.FTCodeSim");
            return true;
        } catch (ClassNotFoundException | LinkageError e) {
            return false;
        }
    }
}
