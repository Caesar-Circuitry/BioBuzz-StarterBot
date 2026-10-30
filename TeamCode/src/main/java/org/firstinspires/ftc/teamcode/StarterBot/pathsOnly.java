//Copied from Brendan's path
package org.firstinspires.ftc.teamcode.StarterBot;
import static com.pedropathing.api.Paths.*;
import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;

public class pathsOnly {
    private final PoseFactory poseFactory = PoseFactory.degrees();

    private final Pose start = poseFactory.of(57, 8.5, 270);
    private final Pose path1 = poseFactory.of(8.5, 8.5, 180);
    private final Pose path1Control1 = poseFactory.of(43.2182, 15.8361, 0);
    private final Pose point2Start = poseFactory.of(8.5, 8.5, 180);
    private final Pose point2 = poseFactory.of(59.8241, 133.6878, 90);
    private final Pose point2Control1 = poseFactory.of(11.8628, 73.8766, 0);
    private final Pose point2Control2 = poseFactory.of(59.9641, 118.2901, 0);
    private final Pose point3Start = poseFactory.of(59.8241, 133.6878, 90);
    private final Pose point3 = poseFactory.of(14.2597, 99.6943, 90);
    private final Pose point3Control1 = poseFactory.of(44.7486, 110.4466, 0);

    public Pose getStartPose() {
        return start;
    }

    public Path path1() {
        return Paths.curve(start, path1Control1, path1).linear(start, path1);
    }

    public Path path2() {
        return Paths.curve(point2Start, point2Control1, point2Control2, point2).linear(point2Start, point2);
    }

    public Path path3() {
        return Paths.curve(point3Start, point3Control1, point3).linear(point3Start, point3);
    }
}
