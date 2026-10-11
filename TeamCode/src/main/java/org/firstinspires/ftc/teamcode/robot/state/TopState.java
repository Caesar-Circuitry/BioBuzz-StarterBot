package org.firstinspires.ftc.teamcode.robot.state;

import org.firstinspires.ftc.teamcode.robot.Robot;

public interface TopState {
    String name();
    void enter(Robot robot);
    void execute(Robot robot);
    void exit(Robot robot);
}
