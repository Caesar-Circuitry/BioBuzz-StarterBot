package org.firstinspires.ftc.teamcode.sim.biobuzz;

/** What the field needs to know about the robot each physics step. Field frame, inches. */
public class RobotInput {
    public double x, y, heading;
    /** Field-frame velocity (in/s) and turn rate (rad/s). */
    public double vx, vy, omega;

    /** Intake effort (-1..1), positive = pulling balls in (SimulatedIntake.power()). */
    public double intakePower;
}
