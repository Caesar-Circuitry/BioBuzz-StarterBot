package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ControlConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.MatchConstants;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.state.AutoHandoff;
import org.firstinspires.ftc.teamcode.robot.state.RobotSnapshot;
import org.psilynx.psikit.core.Logger;

@TeleOp(name = "Competition TeleOp", group = "TeleOp")
/**
 * The driver-controlled OpMode used in matches.
 *
 * Layer: opmodes. This is the only place that knows which button does what.
 * It turns gamepad1 into calls on Robot's intent methods and never touches a
 * subsystem, so changing the button layout cannot break robot behavior.
 */
public class CompetitionTeleOp extends RobotOpMode {

    // Start where auto ended, if an auto ran. Otherwise use the fixed pose.
    @Override protected Pose startPose() {
        return AutoHandoff.get(FieldConstants.TELEOP_START);
    }

    @Override protected String modeName() {
        return "Competition TeleOp";
    }

    // ---- Fields ----

    // Last loop's values, kept to spot the moment something CHANGES.
    private boolean wasReady = false;
    private boolean leftTriggerWasDown = false;

    // ---- Gamepad mapping ----

    @Override protected void handleGamepads() {
        // Stick up reads as negative on a gamepad, so y is flipped: up = forward.
        double forward = deadband(-gamepad1.left_stick_y);
        double strafe = deadband(gamepad1.left_stick_x);
        double turn = deadband(gamepad1.right_stick_x);
        robot.drive(forward, strafe, turn);

        // Toggles use a FRESH press (WasPressed is true for one loop only). A
        // held button read every loop would flip the toggle on every loop and
        // land on a random side.
        if (gamepad1.crossWasPressed()) {
            robot.toggleIntake();
        }
        if (gamepad1.circleWasPressed()) {
            robot.toggleLauncher();
        }
        if (gamepad1.squareWasPressed()) {
            robot.toggleFieldCentric();
        }
        if (gamepad1.leftBumperWasPressed()) {
            robot.unjam();
        }

        // The assist is hold-to-run: press starts it, release ends it, so the
        // driver can take back control just by letting go.
        if (gamepad1.rightBumperWasPressed())  robot.startAssist();
        if (gamepad1.rightBumperWasReleased()) robot.endAssist();

        // Single shot. A trigger is a number, not a button, so there is no
        // WasPressed for it. We make our own fresh press: down now, up last loop.
        boolean leftTriggerDown = gamepad1.left_trigger > ControlConstants.TRIGGER_THRESHOLD;
        if (leftTriggerDown && !leftTriggerWasDown) {
            robot.launch();
        }
        leftTriggerWasDown = leftTriggerDown;

        // Full auto is the opposite: a LEVEL, sent every loop. Shooting lasts
        // exactly as long as the trigger is held.
        robot.setFullAuto(gamepad1.right_trigger > ControlConstants.TRIGGER_THRESHOLD);

        // Raw inputs, before deadband or thresholds, so a log can tell "the
        // driver never pressed it" apart from "the robot ignored it". Buttons
        // are logged as held or not: reading WasPressed a second time here
        // would use up the press.
        Logger.recordOutput("Controls/LeftStickY", (double) gamepad1.left_stick_y);
        Logger.recordOutput("Controls/LeftStickX", (double) gamepad1.left_stick_x);
        Logger.recordOutput("Controls/RightStickX", (double) gamepad1.right_stick_x);
        Logger.recordOutput("Controls/LeftTrigger", (double) gamepad1.left_trigger);
        Logger.recordOutput("Controls/RightTrigger", (double) gamepad1.right_trigger);
        Logger.recordOutput("Controls/Cross", gamepad1.cross);
        Logger.recordOutput("Controls/Circle", gamepad1.circle);
        Logger.recordOutput("Controls/Square", gamepad1.square);
        Logger.recordOutput("Controls/LeftBumper", gamepad1.left_bumper);
        Logger.recordOutput("Controls/RightBumper", gamepad1.right_bumper);
    }

    // ---- Helpers ----

    // Sticks rarely rest at exactly zero. Without this the robot creeps.

    private static double deadband(double v) {
        return Math.abs(v) < ControlConstants.STICK_DEADBAND ? 0 : v;
    }

    // ---- Driver feedback ----

    @Override
    protected void applyFeedback() {
        RobotSnapshot s = robot.snapshot();
        boolean ready = s.readyToLaunch();
        // NOTE: this is a local, so it starts as false every loop. Left as
        // found; listed as a possible bug.
        boolean wasArrived = false;

        // blip only on the moment it becomes ready, and not during full auto
        // UNVERIFIED: rumble(ms) exists in the SDK. Check on the real gamepad that
        // the driver can feel a blip this short.
        if (ready && !wasReady && !s.fullAuto) {
            gamepad1.rumble(ControlConstants.RUMBLE_READY_MS);
        }
        if (s.assistArrived && !wasArrived) gamepad1.rumble(ControlConstants.RUMBLE_ARRIVED_MS);
        wasArrived = s.assistArrived;
        wasReady = ready;
    }

    @Override protected void onModeStart() { robot.startMatch(MatchConstants.TELEOP_SECONDS); }
}
