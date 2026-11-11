package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class LimelightVision {
    private final Limelight3A limelight;
    private LLResult result;

    public LimelightVision(HardwareMap hardwareMap, int pipeline) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(pipeline);
        limelight.start();
    }

    public void update() {
        result = limelight.getLatestResult();
    }

    public boolean hasTarget() {
        return result != null && result.isValid();
    }

    public double getTx() { return hasTarget() ? result.getTx() : 0; }
    public double getTy() { return hasTarget() ? result.getTy() : 0; }
    public double getTa() { return hasTarget() ? result.getTa() : 0; }
    public String getBestClassName() {
        if (!hasTarget()) return "none";
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        if (detections == null || detections.isEmpty()) return "none";
        return detections.get(0).getClassName();
    }

    public void switchPipeline(int pipeline) { limelight.pipelineSwitch(pipeline); }
    public void stop() { limelight.stop(); }
}
