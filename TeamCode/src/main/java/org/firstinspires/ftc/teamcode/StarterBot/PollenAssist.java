package org.firstinspires.ftc.teamcode.StarterBot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.util.Range;

@Configurable
public class PollenAssist {

    public static double TURN_kP = 0.02;
    public static double DRIVE_kP = 0.03;
    public static double TARGET_TY = -15;
    public static double MAX_TURN = 0.4;
    public static double MAX_DRIVE = 0.5;
    public static double TX_DEADBAND = 1.0;
    public static double ALIGN_BEFORE_DRIVE = 15;

    private double turn = 0;
    private double forward = 0;

    public void update(LimelightVision vision) {
        if (!vision.hasTarget()) {
            turn = 0;
            forward = 0;
            return;
        }

        double tx = vision.getTx();
        double ty = vision.getTy();
        if (Math.abs(tx) < TX_DEADBAND) {
            turn = 0;
        } else {
            turn = Range.clip(tx * TURN_kP, -MAX_TURN, MAX_TURN);
        }

        double distanceError = ty - TARGET_TY;
        forward = Range.clip(distanceError * DRIVE_kP, 0, MAX_DRIVE);
        double alignment = Math.max(0, 1 - Math.abs(tx) / ALIGN_BEFORE_DRIVE);
        forward *= alignment;
    }

    public double getTurn()    { return turn; }
    public double getForward() { return forward; }
}
