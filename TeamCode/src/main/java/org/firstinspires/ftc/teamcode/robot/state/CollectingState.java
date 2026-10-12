package org.firstinspires.ftc.teamcode.robot.state;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * Intake running, picking up pollen.
 *
 * Layer: state. The intake is switched on in enter() and off in exit(), so no
 * matter how this state is left, the intake cannot be left running by mistake.
 */
public class CollectingState implements TopState {
    @Override public String name() { return "Collecting"; }

    @Override
    public void enter(Robot robot) {robot.run(robot.intake.on());}

    @Override
    public void execute(Robot robot) {}

    @Override
    public void exit(Robot robot) {robot.run(robot.intake.off());}
}
