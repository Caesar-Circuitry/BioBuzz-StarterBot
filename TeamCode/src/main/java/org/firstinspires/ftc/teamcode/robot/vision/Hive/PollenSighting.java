package org.firstinspires.ftc.teamcode.robot.vision.Hive;

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
