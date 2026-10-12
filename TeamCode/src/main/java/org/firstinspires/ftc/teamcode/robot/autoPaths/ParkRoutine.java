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

/**
 * Autonomous routine: drive from the start pose to the park pose and stay.
 *
 * Layer: auto routines, built on Robot and its subsystems. This class only
 * BUILDS a command; the auto OpMode hands it to the scheduler. Other routines
 * end with this one, so parking is always the last thing an auto does.
 */
public final class ParkRoutine {

    // Logged by the auto OpModes as Autos/Routine.
    public static final String NAME = "Park";

    public static Command build(Robot robot) {
        Follower follower = robot.driveTrain.follower();

        Path path = Paths.line(FieldConstants.AUTO_START, FieldConstants.PARK_POSE);

        Command drive = sequential(
                PedroCommands.follow(follower, path),
                PedroCommands.hold(follower)
        );
        // race = whichever finishes first ends both. "drive" ends with a hold,
        // and if the robot is blocked it may never get there, so the timeout
        // makes sure the routine always moves on.
        // The stop() after it is the cleanup rule: anything that starts the
        // follower on a path also stops it on the way out.
        // UNVERIFIED: Pedro has stop() here (there is no cancel()). Check on the
        // robot that the motors are released at the end of auto.
        return sequential(
                race(drive, waitMs(AutoConstants.PARK_TIMEOUT_MS)),
                instant(() -> follower.stop())
        );
    }

    private ParkRoutine() {}
}
