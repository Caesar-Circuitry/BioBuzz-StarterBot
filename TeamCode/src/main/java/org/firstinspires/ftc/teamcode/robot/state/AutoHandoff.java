package org.firstinspires.ftc.teamcode.robot.state;

import com.pedropathing.math.Pose;

/**
 * Carries the robot's pose from the end of autonomous to the start of TeleOp.
 *
 * Layer: state. The field is static because the auto OpMode object is thrown
 * away when it stops; a static value lives on until the app restarts. Each
 * auto clears it at start, so a pose from an earlier match is never reused
 * by the next auto.
 */
public class AutoHandoff {
    private static Pose pose;

    public static void save(Pose p) {
        pose = p;
    }
    // The fallback covers TeleOp being run on its own, with no auto before it.
    public static Pose get(Pose fallback) {
        return pose != null ? pose : fallback;
    }
    public static void clear() {
        pose = null;
    }

    private AutoHandoff() {}
}
