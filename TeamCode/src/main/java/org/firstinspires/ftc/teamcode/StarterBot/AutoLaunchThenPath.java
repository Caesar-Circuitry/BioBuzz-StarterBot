package org.firstinspires.ftc.teamcode.StarterBot;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "Auto - Launch Then Path", group = "Competition")
public class AutoLaunchThenPath extends AutoBase {

    private final Pose start = p.of(55, 8, -90);
    private final Pose path1 = p.of(10, 96, -90);

    private Path path1() {
        return line(start, path1).constant(path1);
    }

    @Override
    protected Pose startPose() {
        return start;
    }

    @Override
    protected Command routine() {
        return sequential(
                shootAll(),
                label("driving to path1"),
                follow(follower, path1()),
                label("done")
        );
    }
}