package org.firstinspires.ftc.teamcode.StarterBot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Collections;
import java.util.List;

/**
 * One Limelight 3A on a servo mount, used for two jobs:
 *
 *   HIVE     - servo DOWN, camera looks at the floor: Hive Vision (hivevision.dev), a
 *              neural detector that finds yellow pollen, red nectar and blue nectar.
 *              setTarget() picks which of those the robot goes after.
 *   APRILTAG - servo UP, camera looks at the tags under the HIVE cells.
 *
 * The Limelight runs one pipeline at a time, so setPipeline() moves the servo AND switches
 * the pipeline together. While the servo is still travelling the camera is pointing
 * somewhere in between, so every getter reports "nothing" until isReady() is true:
 *   1. the servo has had SETTLE_SECONDS to arrive,
 *   2. the frame really came from the pipeline we asked for (pipelineSwitch() does not wait), and
 *   3. the frame is fresh.
 *
 * The camera position matters only for AprilTag maths, and the servo is always UP in
 * APRILTAG mode. So measure the camera position ONCE with the servo at AprilTagPosition and
 * enter it in the Limelight web page (AprilTag pipeline, robot-space camera pose).
 */
@Configurable
public class LimelightVision {
    public static int HIVE_PIPELINE = 0;
    public static int APRILTAG_PIPELINE = 1;

    public static double HivePosition = 0.48;
    public static double AprilTagPosition = 0.95;
    public static double SETTLE_SECONDS = 0.4;
    public static double MAX_STALENESS_MS = 100;

    public static int CLASS_POLLEN = 0;
    public static int CLASS_RED_NECTAR = 1;
    public static int CLASS_BLUE_NECTAR = 2;
    public static double MIN_CONFIDENCE = 0.30;

    public static int RED_TAG_MIN = 30, RED_TAG_MAX = 37;
    public static int BLUE_TAG_MIN = 38, BLUE_TAG_MAX = 45;

    public enum pipeline {
        HIVE,
        APRILTAG;

        int slot() { return this == HIVE ? HIVE_PIPELINE : APRILTAG_PIPELINE; }
        double servoPosition() { return this == HIVE ? HivePosition : AprilTagPosition; }
    }

    public enum target { POLLEN, NECTAR, BOTH }


    private final Limelight3A limelight;
    private final Servo LLMount;
    private final ElapsedTime sinceSwitch = new ElapsedTime();
    private pipeline selectedPipeline;
    private LLResult result;
    private boolean ready = false;
    private target selectedTarget = target.POLLEN;
    private boolean redAlliance = true;
    private LLResultTypes.DetectorResult best;

    public LimelightVision(HardwareMap hardwareMap) {
        this(hardwareMap, pipeline.HIVE);
    }

    public LimelightVision(HardwareMap hardwareMap, pipeline startPipeline) {
        LLMount = hardwareMap.get(Servo.class, "LLMount");
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        setPipeline(startPipeline);
        limelight.start();
    }

    public void setPipeline(pipeline wanted) {
        if (wanted == selectedPipeline) return;
        selectedPipeline = wanted;
        LLMount.setPosition(wanted.servoPosition());
        limelight.pipelineSwitch(wanted.slot());
        sinceSwitch.reset();
        ready = false;
    }

    public pipeline getPipeline() { return selectedPipeline; }

    public void update() {
        result = limelight.getLatestResult();
        ready = sinceSwitch.seconds() >= SETTLE_SECONDS
                && result != null
                && result.getPipelineIndex() == selectedPipeline.slot()
                && result.getStaleness() <= MAX_STALENESS_MS;
        best = findBest();
    }

    public boolean isReady() { return ready; }

    public void setTarget(target wanted) { selectedTarget = wanted; }
    public target getTarget() { return selectedTarget; }

    public void setAlliance(boolean red) { redAlliance = red; }

    public boolean hasTarget() { return best != null; }

    public double getTx() { return hasTarget() ? best.getTargetXDegrees() : 0; }
    public double getTy() { return hasTarget() ? best.getTargetYDegrees() : 0; }
    public double getTa() { return hasTarget() ? best.getTargetArea() : 0; }

    public String getBestClassName() { return hasTarget() ? best.getClassName() : "none"; }
    public List<LLResultTypes.FiducialResult> getTags() {
        if (!ready || selectedPipeline != pipeline.APRILTAG) return Collections.emptyList();
        List<LLResultTypes.FiducialResult> tags = result.getFiducialResults();
        return tags == null ? Collections.emptyList() : tags;
    }

    public boolean hasTags() { return !getTags().isEmpty(); }
    public int getTagCount() { return getTags().size(); }

    public boolean seesHive(boolean red) { return countHiveTags(red) > 0; }

    public double getHiveTx(boolean red) {
        double sum = 0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : getTags()) {
            if (isHiveTag(tag.getFiducialId(), red)) {
                sum += tag.getTargetXDegrees();
                count++;
            }
        }
        return count == 0 ? 0 : sum / count;
    }

    public Pose3D getBotPose() {
        return hasTags() && result.isValid() ? result.getBotpose() : null;
    }


    public void stop() { limelight.stop(); }

    private LLResultTypes.DetectorResult findBest() {
        if (!ready || selectedPipeline != pipeline.HIVE) return null;
        List<LLResultTypes.DetectorResult> detections = result.getDetectorResults();
        if (detections == null) return null;
        LLResultTypes.DetectorResult biggest = null;
        for (LLResultTypes.DetectorResult d : detections) {
            if (d.getConfidence() < MIN_CONFIDENCE || !wanted(d.getClassId())) continue;
            if (biggest == null || d.getTargetArea() > biggest.getTargetArea()) biggest = d;
        }
        return biggest;
    }

    private boolean wanted(int classId) {
        boolean pollen = classId == CLASS_POLLEN;
        boolean ourNectar = classId == (redAlliance ? CLASS_RED_NECTAR : CLASS_BLUE_NECTAR);
        switch (selectedTarget) {
            case POLLEN: return pollen;
            case NECTAR: return ourNectar;
            default:     return pollen || ourNectar;
        }
    }

    private int countHiveTags(boolean red) {
        int count = 0;
        for (LLResultTypes.FiducialResult tag : getTags()) {
            if (isHiveTag(tag.getFiducialId(), red)) count++;
        }
        return count;
    }

    private static boolean isHiveTag(int id, boolean red) {
        return red ? (id >= RED_TAG_MIN && id <= RED_TAG_MAX)
                : (id >= BLUE_TAG_MIN && id <= BLUE_TAG_MAX);
    }
}