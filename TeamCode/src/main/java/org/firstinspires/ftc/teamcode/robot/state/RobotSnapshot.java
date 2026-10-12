package org.firstinspires.ftc.teamcode.robot.state;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Launcher;

/**
 * A read-only picture of the robot at one moment. OpModes and autos read this
 * so they do not need to reach into each subsystem.
 *
 * Layer: state. Every field is final, so a snapshot can never change after it
 * is made; a new picture means a new object.
 */
public class RobotSnapshot {
    public final Drivetrain.Mode driveMode;
    public final Intake.MODE intakeMode;
    public final Launcher.State launcherState;
    public final boolean launcherEnabled;
    public final boolean fullAuto;
    public final double x, y, headingRad;
    public final String topState;
    public final boolean assistArrived;

    public RobotSnapshot(Drivetrain.Mode driveMode, Intake.MODE intakeMode, Launcher.State launcherState, boolean launcherEnabled, boolean fullAuto, double x, double y, double headingRad, String topState, boolean assistArrived) {
        this.driveMode = driveMode;
        this.intakeMode = intakeMode;
        this.launcherState = launcherState;
        this.launcherEnabled = launcherEnabled;
        this.fullAuto = fullAuto;
        this.x = x;
        this.y = y;
        this.headingRad = headingRad;
        this.topState = topState;
        this.assistArrived = assistArrived;
    }

    public boolean readyToLaunch() {
        return launcherState == Launcher.State.READY;
    }
}
