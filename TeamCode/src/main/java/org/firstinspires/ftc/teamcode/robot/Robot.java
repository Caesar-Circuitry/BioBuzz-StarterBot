package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.skeletonarmy.marrow.LynxUtil;

import org.firstinspires.ftc.teamcode.robot.state.CollectingState;
import org.firstinspires.ftc.teamcode.robot.state.IdleState;
import org.firstinspires.ftc.teamcode.robot.state.LaunchingState;
import org.firstinspires.ftc.teamcode.robot.state.RobotSnapshot;
import org.firstinspires.ftc.teamcode.robot.state.TopState;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Launcher;
import org.psilynx.psikit.core.Logger;

public class Robot {
    public final HardwareMap hMap;
    public final Drivetrain driveTrain;
    public final Intake intake;
    public final Launcher launcher;
    public final RobotSnapshot snapshot;

    private final IdleState idle = new IdleState();
    private final CollectingState collecting = new CollectingState();
    private final LaunchingState Launching = new LaunchingState();
    private TopState topState;

    public Robot(HardwareMap hardwareMap, Pose startPose) {
        hMap = hardwareMap;
        driveTrain = new Drivetrain(this,startPose);
        intake = new Intake(this);
        launcher = new Launcher(this);
        snapshot = new RobotSnapshot(driveTrain.mode(), intake.mode(), launcher.state(), launcher.getEnabled(), launcher.getFullAuto(), driveTrain.follower().pose().x(), driveTrain.follower().pose().y(), driveTrain.follower().pose().heading());
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        Scheduler.schedule(driveTrain.periodic(), intake.periodic(), launcher.periodic());

        topState = idle;
        topState.enter(this);
    }

    public void run(Command command) {
        Scheduler.schedule(command);
    }

    public void toggleIntake(){
        TopState current = (topState == Launching) ? Launching.returnTo() : topState;
        TopState next = (current == collecting) ? idle : collecting;
        if (topState == Launching) Launching.setReturnTo(next);
        else setState(next);
    }

    public void toggleLauncher(){
        run(launcher.toggle());
    }
    public void launch(){
        run(launcher.launch());
        enterLaunching();
    }
    public void setFullAuto(boolean on){
        launcher.setFullAuto(on);
        if (on) enterLaunching();
    }

    public void toggleFieldCentric(){
        driveTrain.toggleFieldCentric();
    }
    public void unjam(){
        run(intake.unjam());
    }

    public void initLoop() {}
    public void start() {}
    public void afterScheduler() {
        Logger.recordOutput("Robot/TopState", topState.name());
        topState.execute(this);
    }
    public void stop() {
        intake.stop();
        launcher.stop();
    }

    public void drive(double forward, double strafe, double turn) {
        driveTrain.drive(forward, strafe, turn);
    }

    public RobotSnapshot robotSnapshot() {
        return snapshot;
    }

    public TopState topState() { return topState; }

    public void setState(TopState next) {
        if (next == topState) return;
        topState.exit(this);
        topState = next;
        topState.enter(this);
    }

    private void enterLaunching() {
        if (topState == Launching) return;
        Launching.setReturnTo(topState);
        setState(Launching);
    }

    public void clearBulkCache() {
        LynxUtil.clearBulkCache(hMap);
    }

    public RobotSnapshot snapshot() {
        return snapshot;
    }
}
