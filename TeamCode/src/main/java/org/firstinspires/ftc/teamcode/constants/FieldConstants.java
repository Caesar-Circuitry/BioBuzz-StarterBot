package org.firstinspires.ftc.teamcode.constants;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;

/**
 * Places on the field (start poses, park pose, pollen presets), in the Pedro frame.
 *
 * Layer: constants. One file per subsystem, so every tunable number has one
 * home and no logic file has a number typed into it.
 */
public final class FieldConstants {
    private static final PoseFactory POSES = PoseFactory.degrees();

    public static final Pose AUTO_START = POSES.of(0,0,0);
    public static final Pose TELEOP_START = POSES.of(0,0,0);

    public static final Pose PARK_POSE = POSES.of(0,0,0);

    public static final Pose[] POLLEN_PRESETS = {};

    private FieldConstants() {}
}
