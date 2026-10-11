package org.firstinspires.ftc.teamcode.robot.autoPaths;

import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.api.Paths;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.pedro.PedroCommands;
import com.pedropathing.paths.Path;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;

public final class ParkRoutine {

    public static Command build(Robot robot) {
        Follower follower = robot.driveTrain.follower();

        Path path = Paths.line(FieldConstants.AUTO_START, FieldConstants.PARK_POSE);

        Command drive = sequential(
                PedroCommands.follow(follower, path),
                PedroCommands.hold(follower)
        );
        return sequential(
                race(drive, waitMs(AutoConstants.PARK_TIMEOUT_MS)),
                instant(() -> follower.stop())
        );
    }
    private ParkRoutine() {}
}
