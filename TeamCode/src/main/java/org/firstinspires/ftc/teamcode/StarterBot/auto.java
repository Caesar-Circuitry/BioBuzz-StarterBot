//path created at Q2 -Joseph
package org.firstinspires.ftc.teamcode.StarterBot;
import static com.pedropathing.api.Paths.curve;
import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@Autonomous(name = "CaesarStarterBot Auto", group = "Competition")
public class auto extends LinearOpMode {
    private final PoseFactory p = PoseFactory.degrees();

    private final Pose start = p.of(58, 8.5, 270);
    private final Pose preload = p.of(57.9642, 11.5297, 270);
    private final Pose cornerStart = p.of(57.9642, 11.5297, -90);
    private final Pose corner = p.of(46.3872, 12.7958, 270);
    private final Pose sweepStart = p.of(46.3872, 12.7958, 180);
    private final Pose sweep = p.of(8.8821, 7.389, 180);
    private final Pose launchStart = p.of(8.8821, 7.389, 180);
    private final Pose launch = p.of(57.3082, 127.098, 90);
    private final Pose launchControl1 = p.of(32, 40, 0);
    private final Pose launchControl2 = p.of(32, 118, 0);
    private final Pose launchControl3 = p.of(31.5224, 95.4786, 0);
    private final Pose parkStart = p.of(57.3082, 127.098, 90);
    private final Pose park = p.of(9.6831, 105.7377, 90);

    private Follower follower;

    @Override
    public void runOpMode() {
        Scheduler.reset();
        follower = Constants.create(hardwareMap);
        follower.setPose(start);
        follower.update();

        telemetry.addLine("Route ready");
        telemetry.update();
        waitForStart();

        if (isStopRequested()) return;

        schedule(autoRoutine());

        while (opModeIsActive()) {
            follower.update();
            Scheduler.execute();

            Pose pose = follower.pose();
            telemetry.addData("x", pose.x());
            telemetry.addData("y", pose.y());
            telemetry.addData("heading", Math.toDegrees(pose.heading()));

            if (follower.currentPath() != null) {
                telemetry.addData("Distance remaining", follower.distanceToEndpoint());
                telemetry.addData("Path number", follower.pathIndex());
            }

            telemetry.update();
        }
    }

    public Command autoRoutine() {
        return sequential(
                follow(follower, preloadPath()),
                follow(follower, cornerPath()),
                follow(follower, sweepPath()),
                follow(follower, launchPath()),
                follow(follower, parkPath())
        );
    }

    public Path preloadPath() {
        return line(start, preload).constant(preload);
    }

    public Path cornerPath() {
        return line(cornerStart, corner).linear(cornerStart, corner);
    }

    public Path sweepPath() {
        return line(sweepStart, sweep).linear(sweepStart, sweep);
    }

    public Path launchPath() {
        return curve(
                launchStart,
                launchControl1,
                launchControl2,
                launchControl3,
                launch
        ).linear(launchStart, launch);
    }

    public Path parkPath() {
        return line(parkStart, park).linear(parkStart, park);
    }
}
