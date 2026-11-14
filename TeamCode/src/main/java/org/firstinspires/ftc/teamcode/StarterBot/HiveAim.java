package org.firstinspires.ftc.teamcode.StarterBot;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Position;


@Configurable
public class HiveAim {
    public static double TURN_kP = 0.02;
    public static double MAX_TURN = 0.5;
    public static double AIM_TOLERANCE_DEG = 2.0;
    public static double AIM_OFFSET_DEG = 0.0;
    public static double DISTANCE_OFFSET_IN = 0.0;

    public enum Source { TAGS, ODOMETRY, NONE }

    private Source source = Source.NONE;
    private double aimErrorDeg = 0;
    private double distanceIn = 0;
    private int tagsUsed = 0;

    private boolean hiveKnown = false;
    private double hiveX = 0, hiveY = 0;
    public void update(LimelightVision vision, boolean redAlliance, Pose robotPose) {
        double sumForward = 0, sumLeft = 0;
        int count = 0;
        for (LLResultTypes.FiducialResult tag : vision.getTags()) {
            if (!isOurHiveTag(tag.getFiducialId(), redAlliance)) continue;
            Position p = tag.getTargetPoseRobotSpace().getPosition().toUnit(DistanceUnit.INCH);
            sumForward += p.x;
            sumLeft += p.y;
            count++;
        }
        tagsUsed = count;

        if (count > 0) {
            double forward = sumForward / count;
            double left = sumLeft / count;
            aimErrorDeg = Math.toDegrees(Math.atan2(left, forward)) + AIM_OFFSET_DEG;
            distanceIn = Math.hypot(forward, left) + DISTANCE_OFFSET_IN;
            source = Source.TAGS;

            if (robotPose != null) {
                double h = robotPose.heading();
                hiveX = robotPose.x() + forward * Math.cos(h) - left * Math.sin(h);
                hiveY = robotPose.y() + forward * Math.sin(h) + left * Math.cos(h);
                hiveKnown = true;
            }
            return;
        }

        if (robotPose != null && hiveKnown) {
            double dx = hiveX - robotPose.x();
            double dy = hiveY - robotPose.y();
            double bearing = AngleUnit.normalizeRadians(Math.atan2(dy, dx) - robotPose.heading());
            aimErrorDeg = Math.toDegrees(bearing) + AIM_OFFSET_DEG;
            distanceIn = Math.hypot(dx, dy) + DISTANCE_OFFSET_IN;
            source = Source.ODOMETRY;
            return;
        }

        source = Source.NONE;
        aimErrorDeg = 0;
        distanceIn = 0;
    }

    public double getTurn() {
        if (source == Source.NONE) return 0;
        return Range.clip(-aimErrorDeg * TURN_kP, -MAX_TURN, MAX_TURN);
    }

    public boolean hasAim()        { return source != Source.NONE; }
    public boolean isAimed()       { return hasAim() && Math.abs(aimErrorDeg) < AIM_TOLERANCE_DEG; }
    public Source getSource()      { return source; }
    public double getAimErrorDeg() { return aimErrorDeg; }
    public double getDistanceIn()  { return distanceIn; }
    public int getTagsUsed()       { return tagsUsed; }
    public boolean isHiveKnown()   { return hiveKnown; }

    public void forgetHive()       { hiveKnown = false; }

    private static boolean isOurHiveTag(int id, boolean red) {
        return red ? (id >= LimelightVision.RED_TAG_MIN && id <= LimelightVision.RED_TAG_MAX)
                : (id >= LimelightVision.BLUE_TAG_MIN && id <= LimelightVision.BLUE_TAG_MAX);
    }
}