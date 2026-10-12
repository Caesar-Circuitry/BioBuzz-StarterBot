package org.firstinspires.ftc.teamcode.robot.state;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.AssistConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * The pollen assist is driving the robot. Lasts while the driver holds the
 * assist button, then goes back to whatever the robot was doing before.
 *
 * Layer: state. PollenAssist plans and follows the path; this state handles
 * the switching around it (drive mode, intake, timeout).
 */
public class AssistState implements TopState {

    // ---- Fields ----

    private TopState returnTo;
    private final ElapsedTime timer = new ElapsedTime();

    // ---- Return target ----

    public void setReturnTo(TopState state) {
        this.returnTo = state;
    }

    public TopState ReturnTo() {
        return returnTo;
    }

    // ---- TopState ----

    @Override public String name() {
        return "Assisting";
    }

    @Override public void enter(Robot robot) {
        timer.reset();
        // PATH mode first, so the drivetrain's periodic loop stops writing stick
        // powers before the follower is given a path.
        robot.driveTrain.usePathMode();
        // The point of the assist is to collect, so the intake runs on the way.
        robot.run(robot.intake.on());
        // Guard: no target means no path. Leave at once so the driver is not
        // stuck in a state where the sticks are ignored.
        if (!robot.assist.start()) robot.endAssist();
    }

    @Override public void execute(Robot robot) {
        robot.assist.update();
        // Safety timeout: give control back even if the button stays held.
        // NOTE: this compares timer.seconds() with a constant named ..._MS
        // (milliseconds). Left as found; listed as a possible bug.
        if (timer.seconds() > AssistConstants.MAX_ASSIST_MS) {
            robot.endAssist();
        }
    }

    @Override public void exit(Robot robot) {
        // Stop the follower on every way out (button release, timeout, no
        // target), or the robot would keep driving the path.
        robot.assist.stop();
        robot.run(robot.intake.off());
    }
}
