package org.firstinspires.ftc.teamcode.robot.autoPaths;

import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.race;
import static com.pedropathing.ivy.groups.Groups.repeat;
import static com.pedropathing.ivy.groups.Groups.sequential;

import com.pedropathing.ivy.Command;

import org.firstinspires.ftc.teamcode.constants.AutoConstants;
import org.firstinspires.ftc.teamcode.constants.LauncherConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.function.BooleanSupplier;

/**
 * Autonomous routine: shoot the preloaded pollen, then park.
 *
 * Layer: auto routines, built on Robot and its subsystems. Every wait in here
 * is raced against a timeout, so a launcher that never gets to speed cannot
 * use up the whole autonomous period: Park always gets its turn.
 */
public final class PreloadRoutine {

    // Logged by the auto OpModes as Autos/Routine.
    public static final String NAME = "Preload";

    public static Command build(Robot robot) {
        BooleanSupplier ready = () -> robot.snapshot.readyToLaunch();

        // One shot: wait until ready (but not forever), ask for the shot, then
        // wait out the feed and the gap so the next request is not dropped.
        Command oneLaunch = sequential(
                race(waitUntil(ready), waitMs(AutoConstants.READY_TIMEOUT_MS)),
                robot.launcher.launch(),
                waitMs(LauncherConstants.QUARTER_TURN_MS + LauncherConstants.MIN_FEED_GAP_MS)
        );

        // The whole shooting phase has its own, longer timeout. Whatever happens
        // above, shooting ends in time to leave room for parking.
        // UNVERIFIED: repeat() is given the same oneLaunch command each time. Check
        // on the robot that all PRELOAD_COUNT shots are attempted, not only one.
        Command launching = race(
                sequential(robot.launcher.enable(), repeat(oneLaunch, AutoConstants.PRELOAD_COUNT)),
                waitMs(AutoConstants.SHOOT_WINDOW_MS)
        );
        // Wheel off before driving: nothing more will be shot in this auto.
        return sequential(launching,
                robot.launcher.disable(),
                ParkRoutine.build(robot));
    }

    private PreloadRoutine() {}
}
