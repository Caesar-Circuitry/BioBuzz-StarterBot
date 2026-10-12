package org.firstinspires.ftc.teamcode.robot.vision.Hive;

/**
 * One pollen seen by the camera: where it is in the image and how old the
 * picture was. A plain data holder with no logic.
 *
 * Layer: vision.
 */
public class PollenSighting {
    public final double txDegrees;
    public final double tyDegrees;
    public final double ageMs;

    public PollenSighting(double txDegrees, double tyDegrees, double ageMs) {
        this.txDegrees = txDegrees;
        this.tyDegrees = tyDegrees;
        this.ageMs = ageMs;
    }
}
