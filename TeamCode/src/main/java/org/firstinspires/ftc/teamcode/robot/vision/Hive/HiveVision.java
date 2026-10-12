package org.firstinspires.ftc.teamcode.robot.vision.Hive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.psilynx.psikit.core.Logger;

/**
 * Reads the Limelight and keeps the biggest (so, most likely nearest) pollen
 * detection from the latest picture.
 *
 * Layer: vision. NOTE: nothing constructs or updates this class yet. Robot
 * does not own one, so the "Vision/" log keys stay empty until it is wired in.
 */
public class HiveVision {

    // ---- Fields ----

    // Not final: it is set back to null when the camera is missing.
    private Limelight3A limelight;
    private PollenSighting closest;

    // ---- Constructor ----

    public HiveVision(Robot r) {
        // A missing or unplugged camera must not crash the whole robot. Without
        // it we lose vision only, so the error is swallowed and connected()
        // reports false.
        try {
            this.limelight = r.hMap.get(Limelight3A.class, VisionConstants.Limelight_NAME);
            limelight.pipelineSwitch(VisionConstants.PIPELINE_INDEX);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
    }

    // ---- Update (call once per loop, from inside the OpMode loop) ----

    public boolean connected() {return limelight != null;}

    public void update() {
        // Forget the last sighting first, so an old one is never reused when
        // this loop has no good picture.
        closest = null;
        Logger.recordOutput("Vision/Connected", limelight != null);
        if (limelight == null) {
            Logger.recordOutput("Vision/HasPollen", false);
            return;
        }

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) {
            Logger.recordOutput("Vision/HasPollen", false);
            return;
        }

        // Logged raw, before the filter, so a log shows how old the picture was
        // even when we threw it away.
        // UNVERIFIED: the SDK docs say getStaleness() is in milliseconds. Check on
        // the robot that this number sits near the camera's frame time.
        Logger.recordOutput("Vision/StalenessMs", (double) result.getStaleness());

        // An old picture shows where the pollen WAS. The robot has moved since,
        // so steering by it would aim at the wrong place.
        if (result.getStaleness() > VisionConstants.MAX_STALENESS_MS) {
            Logger.recordOutput("Vision/HasPollen", false);
            return;
        }

        // Bigger in the picture means closer, so the largest area wins.
        double bestArea = -1;
        for (LLResultTypes.DetectorResult d : result.getDetectorResults()) {
            if (d.getClassId() == VisionConstants.POLLEN_CLASS_ID) continue;
            if (d.getTargetArea() > bestArea) {
                bestArea = d.getTargetArea();
                closest = new PollenSighting(d.getTargetXDegrees(), d.getTargetYDegrees(), result.getStaleness());
            }
        }

        Logger.recordOutput("Vision/HasPollen", closest != null);
        if (closest != null) {
            Logger.recordOutput("Vision/TxDegrees", closest.txDegrees);
            Logger.recordOutput("Vision/TyDegrees", closest.tyDegrees);
        }
    }

    // ---- Getters ----

    // Null when there is no usable sighting this loop.
    public PollenSighting getClosest() {
        return closest;
    }
}
