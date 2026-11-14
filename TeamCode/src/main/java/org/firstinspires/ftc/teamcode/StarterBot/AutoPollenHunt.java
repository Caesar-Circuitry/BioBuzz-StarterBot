package org.firstinspires.ftc.teamcode.StarterBot;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
@Autonomous(name = "Auto - Pollen Hunt", group = "Competition")
public class AutoPollenHunt extends AutoBase {
    public static int BALLS_TO_COLLECT = 4;
    public static double COLLECT_TIMEOUT_SECONDS = 8.0;

    private final Pose start = p.of(55, 8, 90);
    private final Pose launchSpot = p.of(55, 8, 90);
    private final Pose otherSide = p.of(55, 110, 90);
    private final Pose park = p.of(10, 96, -90);

    @Override
    protected Pose startPose() {
        return start;
    }

    @Override
    protected Command routine() {
        return sequential(
                shootAll(),
                label("driving to the other side"),
                follow(follower, line(start, otherSide).linear(start, otherSide)),
                collectPollen(BALLS_TO_COLLECT, COLLECT_TIMEOUT_SECONDS),
                label("driving back to launch"),
                driveTo(launchSpot),
                shootAll(),
                label("parking"),
                driveTo(park),
                label("done")
        );
    }
}