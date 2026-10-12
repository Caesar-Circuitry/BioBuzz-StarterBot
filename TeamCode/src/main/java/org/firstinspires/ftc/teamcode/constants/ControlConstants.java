package org.firstinspires.ftc.teamcode.constants;

/**
 * Driver feel: stick deadband, trigger threshold, rumble lengths.
 *
 * Layer: constants. One file per subsystem, so every tunable number has one
 * home and no logic file has a number typed into it.
 */
public final class ControlConstants {
    public static final double STICK_DEADBAND = 0.05;
    public static final double TRIGGER_THRESHOLD = 0.5;

    public static final double FIELD_HEADING_OFFSET_RAD = 0.0;

    public static final int RUMBLE_READY_MS = 150;

    public static final int RUMBLE_ARRIVED_MS = 200;   // UNMEASURED

    private ControlConstants() {}
}
