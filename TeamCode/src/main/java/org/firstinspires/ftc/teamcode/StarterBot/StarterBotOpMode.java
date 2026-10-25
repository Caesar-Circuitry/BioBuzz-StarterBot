package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.core.rlog.RLOGWriter;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

import java.text.SimpleDateFormat;
import java.util.Date;
import java.util.List;
import java.util.Locale;

/**
 * Shared base for StarterBot OpModes: PsiKit logging set up the same way as
 * RobotOpMode on the Cohen-Testing branch.
 *
 * PsiKitIterativeOpMode starts the logger in init (and wraps hardwareMap so
 * every motor/servo/sensor is logged under /HardwareMap/...), logs gamepads,
 * and calls the onPsiKit... hooks below each loop. Subclasses fill in the hooks
 * and call the log helpers at the end of their loop.
 *
 * Log keys used across StarterBot (same names as the branch):
 *   Robot/...     mode, loop time, battery
 *   Controls/...  raw gamepad inputs, before any processing
 *   Drive/...     requested stick values vs what each drive motor did
 *   Intake/...    mode, requested powers vs measured
 *   Shooter/...   mode, requested powers vs measured flywheel speed
 *   Auto/...      autonomous progress
 */
public abstract class StarterBotOpMode extends PsiKitIterativeOpMode {

    // ---- Fields ----

    private final ElapsedTime loopTimer = new ElapsedTime();
    private VoltageSensor battery;

    // ---- Hooks for subclasses ----

    /** Name written into the log metadata and Robot/Mode. */
    protected abstract String modeName();

    // ---- PsiKit lifecycle ----

    // Port 0 tells PsiKit not to start the live server at all. The simulator always
    // needs it: AdvantageScope's "Connect to Simulator" reads from it.
    @Override protected int getRlogPort() {
        if (LoggingConstants.IN_SIMULATOR) return LoggingConstants.LIVE_SERVER_PORT;
        return LoggingConstants.LIVE_SERVER_ENABLED ? LoggingConstants.LIVE_SERVER_PORT : 0;
    }

    /**
     * Where PsiKit writes the log file. On the robot that's PsiKit's default,
     * /sdcard/FIRST/PsiKit/. On a laptop /sdcard doesn't exist (PsiKit then prints
     * "must start RLOGWriter before using append data" every loop), so the sim
     * writes to TeamCode/sim-logs/ instead.
     *
     * No @Override on purpose: this overrides PsiKitIterativeOpMode's rlogFolder
     * property when the PsiKit version has it, and is harmlessly unused otherwise
     * (see usesSimFolderOverride()).
     */
    protected String getRlogFolder() {
        return LoggingConstants.IN_SIMULATOR ? LoggingConstants.SIM_LOG_FOLDER : "/sdcard/FIRST/PsiKit/";
    }

    // Runs once, before the logger starts. Metadata is written at the top of the
    // log file, so months later a log still says which settings made it.
    @Override protected void onPsiKitConfigureLogging() {
        if (LoggingConstants.IN_SIMULATOR && !usesSimFolderOverride()) {
            // Older PsiKit without the rlogFolder setting: keep a copy next to TeamCode anyway.
            String stamp = new SimpleDateFormat("yyyyMMdd_HHmmss", Locale.US).format(new Date());
            Logger.addDataReceiver(new RLOGWriter(LoggingConstants.SIM_LOG_FOLDER,
                    modeName().replaceAll("[^A-Za-z0-9]+", "_") + "_" + stamp + ".rlog"));
        }
        Logger.recordMetadata("Robot", "StarterBot");
        Logger.recordMetadata("Mode", modeName());
        Logger.recordMetadata("Simulator", String.valueOf(LoggingConstants.IN_SIMULATOR));
        recordConstants();
    }

    @Override protected void onPsiKitInit() {
        battery = firstVoltageSensor();
        loopTimer.reset();
    }

    // ---- Log helpers (call once per loop) ----

    /** Robot/... : what mode we're in, how long the loop took, battery voltage. */
    protected void logRobot() {
        Logger.recordOutput("Robot/Mode", modeName());
        // Time since this line ran last loop. The first value includes init, so ignore it.
        Logger.recordOutput("Robot/LoopTimeMs", loopTimer.milliseconds());
        loopTimer.reset();
        // A sagging battery explains a slow flywheel better than anything else.
        if (battery != null) Logger.recordOutput("Robot/BatteryVolts", battery.getVoltage());
    }

    /**
     * Controls/... : raw gamepad1 inputs, before any processing, so a log can tell
     * "the driver never pressed it" apart from "the robot ignored it".
     */
    protected void logControls() {
        Logger.recordOutput("Controls/LeftStickX", (double) gamepad1.left_stick_x);
        Logger.recordOutput("Controls/LeftStickY", (double) gamepad1.left_stick_y);
        Logger.recordOutput("Controls/RightStickX", (double) gamepad1.right_stick_x);
        Logger.recordOutput("Controls/LeftTrigger", (double) gamepad1.left_trigger);
        Logger.recordOutput("Controls/RightTrigger", (double) gamepad1.right_trigger);
        Logger.recordOutput("Controls/A", gamepad1.a);
        Logger.recordOutput("Controls/B", gamepad1.b);
        Logger.recordOutput("Controls/X", gamepad1.x);
        Logger.recordOutput("Controls/Y", gamepad1.y);
        Logger.recordOutput("Controls/LeftBumper", gamepad1.left_bumper);
        Logger.recordOutput("Controls/RightBumper", gamepad1.right_bumper);
    }

    // ---- Helpers ----

    // Read from the constants class, never typed in again here, so the log cannot
    // disagree with the code that ran.
    private void recordConstants() {
        Logger.recordMetadata("Shooter/StandardFlywheelPower", String.valueOf(constants.LAUNCH_STANDARD_FLYWHEEL_POWER));
        Logger.recordMetadata("Shooter/StandardWindmillPower", String.valueOf(constants.LAUNCH_STANDARD_WINDMILL_POWER));
        Logger.recordMetadata("Shooter/VariableFlywheelPower", String.valueOf(constants.LAUNCH_VARIABLE_FLYWHEEL_POWER));
        Logger.recordMetadata("Shooter/VariableWindmillPower", String.valueOf(constants.LAUNCH_VARIABLE_WINDMILL_POWER));
        Logger.recordMetadata("Intake/MotorPower", String.valueOf(constants.INTAKE_MOTOR_POWER));
        Logger.recordMetadata("Intake/ServoPower", String.valueOf(constants.INTAKE_SERVO_POWER));
        Logger.recordMetadata("Logging/LiveServerEnabled", String.valueOf(LoggingConstants.LIVE_SERVER_ENABLED));
        Logger.recordMetadata("Logging/LiveServerPort", String.valueOf(LoggingConstants.LIVE_SERVER_PORT));
    }

    /** True if this PsiKit version has the rlogFolder setting that getRlogFolder() overrides. */
    private static boolean usesSimFolderOverride() {
        try {
            PsiKitIterativeOpMode.class.getDeclaredMethod("getRlogFolder");
            return true;
        } catch (NoSuchMethodException e) {
            return false;
        }
    }

    // Survives a missing voltage sensor instead of crashing init.
    private VoltageSensor firstVoltageSensor() {
        try {
            List<VoltageSensor> sensors = hardwareMap.getAll(VoltageSensor.class);
            return sensors.isEmpty() ? null : sensors.get(0);
        } catch (RuntimeException e) {
            return null;
        }
    }
}
