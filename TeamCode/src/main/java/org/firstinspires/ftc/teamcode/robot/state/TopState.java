package org.firstinspires.ftc.teamcode.robot.state;

import org.firstinspires.ftc.teamcode.robot.Robot;

/**
 * One top-level thing the robot can be doing (Idle, Collecting, Launching,
 * Assisting). Robot holds exactly one of these at a time.
 *
 * Layer: state (opmodes -> Robot -> subsystems/assist/state). States may
 * call Robot and subsystems. Subsystems must never import this package, so
 * a subsystem can be understood without knowing the game plan.
 *
 * enter()   runs once when the state becomes current
 * execute() runs every loop while it is current
 * exit()    runs once when it is replaced, to undo what enter() switched on
 */
public interface TopState {
    String name();
    void enter(Robot robot);
    void execute(Robot robot);
    void exit(Robot robot);
}
