package org.firstinspires.ftc.teamcode.robot.state;

import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.AssistConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class AssistState implements TopState {
    private TopState returnTo;
    private final ElapsedTime timer = new ElapsedTime();

    public void setReturnTo(TopState state) {
        this.returnTo = state;
    }
    public TopState ReturnTo() {
        return returnTo;
    }
    @Override public String name() {
        return "Assisting";
    }

    @Override public void enter(Robot robot) {
        timer.reset();
        robot.driveTrain.usePathMode();
        robot.run(robot.intake.on());
        if (!robot.assist.start()) robot.endAssist();
    }

    @Override public void execute(Robot robot) {
        robot.assist.update();
        if (timer.seconds() > AssistConstants.MAX_ASSIST_MS) { // 10 second timeout
            robot.endAssist();
        }
    }

    @Override public void exit(Robot robot) {
        robot.assist.stop();
        robot.run(robot.intake.off());
    }
}
