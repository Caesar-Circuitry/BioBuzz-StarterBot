package org.firstinspires.ftc.teamcode.StarterBot;

/**
 * Every StarterBot number in one place, so the code and the log metadata
 * (StarterBotOpMode.recordConstants) can never disagree.
 */
public final class constants {
    private constants() {}

    // ---- Hardware names (must match the Robot Configuration) ----
    public static final String FL_DRIVE = "FLDrive";
    public static final String FR_DRIVE = "FRDrive";
    public static final String RL_DRIVE = "RLDrive";
    public static final String RR_DRIVE = "RRDrive";
    public static final String INTAKE_MOTOR = "intake";
    public static final String INTAKE_SERVO_1 = "intakeServo1";
    public static final String INTAKE_SERVO_2 = "intakeServo2";
    public static final String LAUNCHER_MOTOR = "launcher";
    public static final String LAUNCHER_SERVO = "launcherServo";

    // ---- Launcher ----
    /** "IDK what a 'standard' launch speed is" -- the sim suggests ~0.84 (about 2350 ticks/s) from its default start spot. */
    public static final double LAUNCH_STANDARD_FLYWHEEL_POWER = 1.0;
    public static final double LAUNCH_STANDARD_WINDMILL_POWER = 1.0;
    public static final double LAUNCH_VARIABLE_FLYWHEEL_POWER = 0.5;
    public static final double LAUNCH_VARIABLE_WINDMILL_POWER = 0.5;

    // ---- Intake ----
    public static final double INTAKE_MOTOR_POWER = 1.0;
    public static final double INTAKE_SERVO_POWER = 1.0;
}
