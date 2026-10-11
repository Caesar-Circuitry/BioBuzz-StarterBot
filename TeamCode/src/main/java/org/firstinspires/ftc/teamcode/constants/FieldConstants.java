package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

public final class FieldConstants {
    private static final PoseFactory POSES = PoseFactory.degrees();

    public static final Pose AUTO_START = POSES.of(0,0,0);
    public static final Pose TELEOP_START = POSES.of(0,0,0);

    public static final Pose PARK_POSE = POSES.of(0,0,0);

    private FieldConstants() {}
}
