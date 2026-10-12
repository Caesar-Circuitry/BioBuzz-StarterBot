package org.firstinspires.ftc.teamcode.opmodes.utility;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Utility;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.constants.LauncherConstants;
import org.firstinspires.ftc.teamcode.constants.LoggingConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;

/**
 * A checklist to run in the queue before a match: is everything plugged in,
 * is the battery charged, and are the numbers that must be measured set?
 *
 * Layer: opmodes (utility). It extends the plain SDK OpMode and does NOT use
 * Robot or PsiKit, on purpose: if one device is missing, Robot would crash in
 * its constructor and could not tell us which one. So this OpMode never
 * starts the logger, and nothing here may call Logger.
 */
@Utility(name = "Pre-Match Check")
public class PreMatchCheck extends OpMode {

    // ---- Fields ----

    private Follower follower;

    // ---- OpMode lifecycle ----

    @Override
    public void init() {
        follower = Constants.create(hardwareMap);
    }

    // Everything is in init_loop so the checks update live while someone
    // plugs things in. Nothing moves, because START is never needed.
    @Override
    public void init_loop() {
        follower.update();
        Pose p = follower.pose();

        line("Launcher motor", hasMotor(LauncherConstants.LAUNCHER_NAME));
        line("Windmill servo", hasServo(LauncherConstants.WINDMILL_NAME));
        line("Intake motor", hasMotor(IntakeConstants.MOTOR_NAME));
        line("Intake Left servo", hasServo(IntakeConstants.LEFT_SERVO_NAME));
        line("Intake Right servo", hasServo(IntakeConstants.RIGHT_SERVO_NAME));

        line("Pinpoint pose valid", !Double.isNaN(p.x()) && !Double.isNaN(p.y()));
        telemetry.addData("Pose (in, in, deg)", "%.1f, %.1f, %.1f",
                p.x(), p.y(), Math.toDegrees(p.heading()));

        double volts = hardwareMap.voltageSensor.iterator().next().getVoltage();
        telemetry.addData("Battery", "%.2f V  %s", volts, volts >= 12.0 ? "PASS" : "LOW");

        // These are the values that start at 0 ("UNMEASURED"). The robot still
        // runs with them unset, it just will not shoot, so they are easy to miss.
        notSet("Launcher target speed", LauncherConstants.TARGET_SPEED > 0);
        notSet("Launcher tolerance",    LauncherConstants.ENTER_TOLERANCE > 0);
        notSet("Windmill quarter turn", LauncherConstants.QUARTER_TURN_MS > 0);
        notSet("Launcher kF",           LauncherConstants.KF > 0);
        notSet("Pollen Presets", FieldConstants.POLLEN_PRESETS.length > 0);

        line("Live log server OFF", !LoggingConstants.LIVE_SERVER_ENABLED);
        telemetry.update();
    }

    @Override
    public void loop() {

    }

    // ---- Helpers ----

    private void line(String name, boolean ok) { telemetry.addData(name, ok ? "PASS" : "FAIL"); }
    private void notSet(String name, boolean set) { telemetry.addData(name, set ? "set" : "NOT SET"); }

    // hardwareMap.get throws when the name is not in the robot configuration.
    // Here that is the answer we are looking for, not an error.
    private boolean hasMotor(String n) {
        try { hardwareMap.get(DcMotorEx.class, n); return true; } catch (Exception e) { return false; }
    }
    private boolean hasServo(String n) {
        try { hardwareMap.get(CRServo.class, n); return true; } catch (Exception e) { return false; }
    }
}