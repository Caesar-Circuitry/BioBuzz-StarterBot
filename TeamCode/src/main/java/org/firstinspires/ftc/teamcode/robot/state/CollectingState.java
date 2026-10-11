package org.firstinspires.ftc.teamcode.robot.state;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class CollectingState implements TopState {
    @Override public String name() { return "Collecting"; }

    @Override
    public void enter(Robot robot) {robot.run(robot.intake.on());}

    @Override
    public void execute(Robot robot) {}

    @Override
    public void exit(Robot robot) {robot.run(robot.intake.off());}
}