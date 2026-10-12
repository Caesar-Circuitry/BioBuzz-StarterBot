package org.firstinspires.ftc.teamcode.robot.state;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Launcher;

/**
 * A shot (or a full-auto burst) is in progress. A short-lived state: it
 * remembers what the robot was doing before and goes back to it afterwards.
 *
 * Layer: state. The Launcher subsystem does the real work; this state only
 * waits for it to finish.
 */
public class LaunchingState implements TopState {

    // ---- Fields ----

    private TopState returnTo;
    private int loops;

    // ---- Return target ----

    public void setReturnTo(TopState returnTo) {
        this.returnTo = returnTo;
    }
    public TopState returnTo() {
        return returnTo;
    }

    // ---- TopState ----

    @Override
    public String name() {
        return "Launching";
    }

    @Override
    public void enter(Robot robot) {loops = 0;}

    @Override
    public void execute(Robot robot) {
        loops++;
        Launcher.State s = robot.snapshot.launcherState;
        boolean busy = s == Launcher.State.FEEDING || s == Launcher.State.RECOVERING;
        boolean holdingFullAuto = robot.snapshot.fullAuto;

        // Wait a few loops before allowing an exit. The launch request is a
        // command, so the launcher does not show FEEDING until the scheduler has
        // run it. Without the wait, "not busy" would be true straight away and
        // this state would leave before the shot began.
        if (loops > 2 && !busy && !holdingFullAuto) {
            robot.setState(returnTo);
        }
    }

    @Override
    public void exit(Robot robot) {}
}
