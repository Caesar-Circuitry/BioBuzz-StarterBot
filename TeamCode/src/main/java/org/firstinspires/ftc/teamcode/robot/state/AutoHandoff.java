package org.firstinspires.ftc.teamcode.robot.state;

import com.pedropathing.math.Pose;

public class AutoHandoff {
    private static Pose pose;

    public static void save(Pose p) {
        pose = p;
    }
    public static Pose get(Pose fallback) {
        return pose != null ? pose : fallback;
    }
    public static void clear() {
        pose = null;
    }

    private AutoHandoff() {}
}
