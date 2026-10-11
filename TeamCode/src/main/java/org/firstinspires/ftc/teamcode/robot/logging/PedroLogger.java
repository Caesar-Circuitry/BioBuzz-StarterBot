package org.firstinspires.ftc.teamcode.robot.logging;

import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerLog;
import com.pedropathing.math.Pose;

import org.psilynx.psikit.core.Logger;

import java.util.Map;

public class PedroLogger {
    private PedroLogger() {}

    public static void logPose(Pose pose) {
        Logger.recordOutput("Drive/Pose/XInches",pose.x());
        Logger.recordOutput("Drive/Pose/YInches",pose.y());
        Logger.recordOutput("Drive/Pose/HeadingRadians",pose.heading());
        Logger.recordOutput("Drive/Pose/HeadingDegrees",Math.toDegrees(pose.heading()));
    }

    public static void logFollower(FollowerLog log) {
        logMap("Drive/Follower/Drivetrain", log.drivetrain());
        logMap("Drive/Follower/Algorithm", log.algorithm());
        logMap("Drive/Follower/Localizer", log.localizer());
        logMap("Drive/Follower/FollowState", log.followState());
    }

    private static void logMap(String prefix, Map<String, Object> map) {
        for (Map.Entry<String, Object> e : map.entrySet()) {
            String key = prefix + "/" + e.getKey();
            Object v = e.getValue();
            if (v instanceof Number)       Logger.recordOutput(key, ((Number) v).doubleValue());
            else if (v instanceof Boolean) Logger.recordOutput(key, ((Boolean) v).booleanValue());
            else                           Logger.recordOutput(key, String.valueOf(v));
        }
    }
}
