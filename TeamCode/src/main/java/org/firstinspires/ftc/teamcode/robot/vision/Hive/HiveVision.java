package org.firstinspires.ftc.teamcode.robot.vision.Hive;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.constants.VisionConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class HiveVision {
    private Limelight3A limelight;
    private PollenSighting closest;

    public HiveVision(Robot r) {
        try {
            this.limelight = r.hMap.get(Limelight3A.class, VisionConstants.Limelight_NAME);
            limelight.pipelineSwitch(VisionConstants.PIPELINE_INDEX);
            limelight.start();
        } catch (Exception e) {
            limelight = null;
        }
    }
    public boolean connected() {return limelight != null;}

    public void update() {
        closest = null;
        if (limelight == null) return;

        LLResult result = limelight.getLatestResult();
        if (result == null || !result.isValid()) return;
        if (result.getStaleness() > VisionConstants.MAX_STALENESS_MS) return;

        double bestArea = -1;
        for (LLResultTypes.DetectorResult d : result.getDetectorResults()) {
            if (d.getClassId() == VisionConstants.POLLEN_CLASS_ID) continue;
            if (d.getTargetArea() > bestArea) {
                bestArea = d.getTargetArea();
                closest = new PollenSighting(d.getTargetXDegrees(), d.getTargetYDegrees(), result.getStaleness());
            }
        }
    }

    public PollenSighting getClosest() {
        return closest;
    }
}
