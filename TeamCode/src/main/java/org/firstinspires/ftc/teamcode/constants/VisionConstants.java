package org.firstinspires.ftc.teamcode.constants;

/**
 * Limelight name, pipeline and filters for HiveVision.
 *
 * Layer: constants. One file per subsystem, so every tunable number has one
 * home and no logic file has a number typed into it.
 */
public final class VisionConstants {
    public static final String Limelight_NAME = "limelight";
    public static final int PIPELINE_INDEX = 0;
    public static final int POLLEN_CLASS_ID = 0;
    public static final double MAX_STALENESS_MS = 120;

    private VisionConstants() {}
}
