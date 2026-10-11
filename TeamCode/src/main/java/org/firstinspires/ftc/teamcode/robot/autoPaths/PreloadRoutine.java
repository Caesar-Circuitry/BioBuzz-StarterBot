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

public final class PreloadRoutine {

    public static Command build(Robot robot){
        BooleanSupplier ready = () -> robot.snapshot.readyToLaunch();

        Command oneLaunch = sequential(
                race(waitUntil(ready), waitMs(AutoConstants.READY_TIMEOUT_MS)),
                robot.launcher.launch(),
                waitMs(LauncherConstants.QUARTER_TURN_MS + LauncherConstants.MIN_FEED_GAP_MS)
        );

        Command Launching = race(
                sequential(robot.launcher.enable(), repeat(oneLaunch, AutoConstants.PRELOAD_COUNT)),
                waitMs(AutoConstants.SHOOT_WINDOW_MS)
        );
        return sequential(Launching,
                robot.launcher.disable(),
                ParkRoutine.build(robot));
    }
    private PreloadRoutine() {}
}
