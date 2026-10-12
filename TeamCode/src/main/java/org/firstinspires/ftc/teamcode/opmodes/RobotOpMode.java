package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.constants.AssistConstants;
import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.ControlConstants;
import org.firstinspires.ftc.teamcode.constants.LauncherConstants;
import org.firstinspires.ftc.teamcode.constants.LoggingConstants;
import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

/**
 * The shared base for every match OpMode (TeleOp and the autos).
 *
 * Layer: opmodes, the top one (opmodes -> Robot -> subsystems/assist/state).
 * It fixes the order of each loop in one place, so no OpMode can get it
 * wrong: read gamepads, run the scheduler, let the Robot react, give feedback.
 * Subclasses only fill in the hooks at the bottom.
 */
public abstract class RobotOpMode extends PsiKitIterativeOpMode {

    // ---- Fields ----

    protected Robot robot;

    // ---- PsiKit lifecycle ----

    // Port 0 tells PsiKit not to start the live server at all.
    @Override protected int getRlogPort() {return LoggingConstants.LIVE_SERVER_ENABLED ? LoggingConstants.LIVE_SERVER_PORT : 0;}

    // Runs once, before the logger starts. Metadata is written at the top of
    // the log file, so months later a log still says which settings made it.
    @Override protected void onPsiKitConfigureLogging() {
        Logger.recordMetadata("Robot", "StarterBot");
        Logger.recordMetadata("Mode", modeName());
        recordConstants();
    }

    @Override protected void onPsiKitInit() {
        // reset() comes BEFORE "new Robot". The scheduler is static, so it still
        // holds the commands of the OpMode that ran before this one. The Robot
        // constructor schedules the periodic commands, so resetting after it
        // would delete them and nothing on the robot would run.
        Scheduler.reset();
        robot = new Robot(hardwareMap, startPose());
    }

    @Override protected void onPsiKitInitLoop() {robot.initLoop();}

    @Override protected void onPsiKitStart() {
        robot.start();
        onModeStart();
    }

    // The order matters:
    // 1. handleGamepads  turns buttons into requests
    // 2. execute         runs every command ONCE (twice would double every timer
    //                    step and motor write in a loop)
    // 3. afterScheduler  lets the top state react to what the subsystems just did
    // 4. applyFeedback   rumble and telemetry, using this loop's results
    @Override protected void onPsiKitLoop() {
        handleGamepads();
        Scheduler.execute();
        robot.afterScheduler();
        applyFeedback();
        Logger.recordOutput("Robot/Mode", modeName());
    }

    // Guard: stop can arrive when init failed part way, before the robot exists.
    @Override protected void onPsiKitStop() {
        if (robot != null) {
            robot.stop();
        }
    }

    // ---- Hooks for subclasses ----

    protected abstract Pose startPose();
    protected abstract String modeName();

    protected void handleGamepads() {}     // Blocks 3 and 5
    protected void applyFeedback() {}      // Block 5
    protected void onModeStart() {}        // Block 6

    // ---- Helpers ----

    // The values are read from the constants classes, never typed in again
    // here, so the log cannot disagree with the code that ran.
    // Camera geometry is missing: there are no constants for it yet.
    private void recordConstants() {
        Logger.recordMetadata("Shooter/TargetSpeedTicksPerSec", String.valueOf(LauncherConstants.TARGET_SPEED));
        Logger.recordMetadata("Shooter/EnterToleranceTicksPerSec", String.valueOf(LauncherConstants.ENTER_TOLERANCE));
        Logger.recordMetadata("Shooter/LeaveToleranceTicksPerSec", String.valueOf(LauncherConstants.LEAVE_TOLERANCE));
        Logger.recordMetadata("Shooter/MaxPower", String.valueOf(LauncherConstants.MAX_POWER));
        Logger.recordMetadata("Shooter/Pid/KP", String.valueOf(LauncherConstants.KP));
        Logger.recordMetadata("Shooter/Pid/KI", String.valueOf(LauncherConstants.KI));
        Logger.recordMetadata("Shooter/Pid/KD", String.valueOf(LauncherConstants.KD));
        Logger.recordMetadata("Shooter/Pid/KF", String.valueOf(LauncherConstants.KF));
        Logger.recordMetadata("Shooter/Pid/ILimit", String.valueOf(LauncherConstants.I_LIMIT));
        Logger.recordMetadata("Shooter/WindmillPower", String.valueOf(LauncherConstants.WINDMILL_POWER));
        Logger.recordMetadata("Shooter/QuarterTurnMs", String.valueOf(LauncherConstants.QUARTER_TURN_MS));
        Logger.recordMetadata("Shooter/MinFeedGapMs", String.valueOf(LauncherConstants.MIN_FEED_GAP_MS));

        Logger.recordMetadata("Vision/PipelineIndex", String.valueOf(VisionConstants.PIPELINE_INDEX));
        Logger.recordMetadata("Vision/PollenClassId", String.valueOf(VisionConstants.POLLEN_CLASS_ID));
        Logger.recordMetadata("Vision/MaxStalenessMs", String.valueOf(VisionConstants.MAX_STALENESS_MS));

        Logger.recordMetadata("Assist/MaxAssistMs", String.valueOf(AssistConstants.MAX_ASSIST_MS));
        Logger.recordMetadata("Assist/IntakeWidth", String.valueOf(AssistConstants.INTAKE_WIDTH));
        Logger.recordMetadata("Assist/RobotWidth", String.valueOf(AssistConstants.ROBOT_WIDTH));
        Logger.recordMetadata("Assist/RobotHeight", String.valueOf(AssistConstants.ROBOT_HEIGHT));
        Logger.recordMetadata("Assist/Clearance", String.valueOf(AssistConstants.CLEARANCE));

        Logger.recordMetadata("Autos/ParkTimeoutMs", String.valueOf(AutoConstants.PARK_TIMEOUT_MS));
        Logger.recordMetadata("Autos/ReadyTimeoutMs", String.valueOf(AutoConstants.READY_TIMEOUT_MS));
        Logger.recordMetadata("Autos/ShootWindowMs", String.valueOf(AutoConstants.SHOOT_WINDOW_MS));
        Logger.recordMetadata("Autos/PreloadCount", String.valueOf(AutoConstants.PRELOAD_COUNT));

        Logger.recordMetadata("Controls/StickDeadband", String.valueOf(ControlConstants.STICK_DEADBAND));
        Logger.recordMetadata("Controls/TriggerThreshold", String.valueOf(ControlConstants.TRIGGER_THRESHOLD));
        Logger.recordMetadata("Controls/FieldHeadingOffsetRad", String.valueOf(ControlConstants.FIELD_HEADING_OFFSET_RAD));
    }
}
