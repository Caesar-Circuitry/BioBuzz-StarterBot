package org.firstinspires.ftc.teamcode.robot.state;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Launcher;

public class RobotSnapshot {
    public final Drivetrain.Mode driveMode;
    public final Intake.MODE intakeMode;
    public final Launcher.State launcherState;
    public final boolean launcherEnabled;
    public final boolean fullAuto;
    public final double x, y, headingRad;

    public RobotSnapshot(Drivetrain.Mode driveMode, Intake.MODE intakeMode, Launcher.State launcherState, boolean launcherEnabled, boolean fullAuto, double x, double y, double headingRad) {
        this.driveMode = driveMode;
        this.intakeMode = intakeMode;
        this.launcherState = launcherState;
        this.launcherEnabled = launcherEnabled;
        this.fullAuto = fullAuto;
        this.x = x;
        this.y = y;
        this.headingRad = headingRad;
    }

    public boolean readyToLaunch() {
        return launcherState == Launcher.State.READY;
    }
}
