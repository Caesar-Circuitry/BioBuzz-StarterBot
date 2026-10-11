package org.firstinspires.ftc.teamcode.robot.state;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.subsystems.Launcher;

public class LaunchingState implements TopState{
    private TopState returnTo;
    private int loops;
    public void setReturnTo (TopState returnTo) {
        this.returnTo = returnTo;
    }
    public TopState returnTo() {
        return returnTo;
    }
    @Override
    public String name() {
        return "Launching";
    }

    @Override
    public void enter(Robot robot) {loops = 0;}

    @Override
    public void execute(Robot robot) {
        loops++;
        Launcher.State s = robot.snapshot.launcherState;
        boolean busy = s == Launcher.State.FEEDING || s == Launcher.State.RECOVERING;
        boolean holdingFullAuto = robot.snapshot.fullAuto;

        if(loops >2 && !busy && !holdingFullAuto) {
            robot.setState(returnTo);
        }
    }

    @Override
    public void exit(Robot robot) {}
}
