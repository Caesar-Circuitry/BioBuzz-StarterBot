package org.firstinspires.ftc.teamcode.robot.state;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Nothing running. The driver can still drive; the intake is off.
 *
 * Layer: state. All three methods are empty on purpose: the state before this
 * one switches its own things off in its exit().
 */
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
