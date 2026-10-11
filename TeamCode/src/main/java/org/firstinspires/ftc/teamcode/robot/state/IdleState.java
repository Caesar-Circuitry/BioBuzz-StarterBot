package org.firstinspires.ftc.teamcode.robot.state;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class IdleState implements TopState {
    @Override
    public String name() {
        return "Idle";
    }

    @Override
    public void enter(Robot robot) {}

    @Override
    public void execute(Robot robot) {}

    @Override
    public void exit(Robot robot) {}
}
