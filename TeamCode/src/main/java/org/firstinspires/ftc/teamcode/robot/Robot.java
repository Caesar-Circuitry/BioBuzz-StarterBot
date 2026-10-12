package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.skeletonarmy.marrow.LynxUtil;
import com.skeletonarmy.marrow.TimerEx;

import org.firstinspires.ftc.teamcode.robot.assist.PollenAssist;
import org.firstinspires.ftc.teamcode.robot.state.AssistState;
import org.firstinspires.ftc.teamcode.robot.state.CollectingState;
import org.firstinspires.ftc.teamcode.robot.state.IdleState;
import org.firstinspires.ftc.teamcode.robot.state.LaunchingState;
import org.firstinspires.ftc.teamcode.robot.state.RobotSnapshot;
import org.firstinspires.ftc.teamcode.robot.state.TopState;
import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Launcher;
import org.psilynx.psikit.core.Logger;

/**
 * The whole robot in one object. It owns the subsystems and the top-level
 * state machine (Idle, Collecting, Launching, Assisting).
 *
 * Layer: the middle one (opmodes -> Robot -> subsystems/assist/state).
 * OpModes call the "intent" methods here (toggleIntake, launch, startAssist...)
 * and never touch a motor. That keeps button mapping in the OpMode and robot
 * behavior in here, so a new OpMode cannot forget a safety rule.
 */
public class Robot {

    // ---- Fields ----

    public final HardwareMap hMap;
    public final Drivetrain driveTrain;
    public final Intake intake;
    public final Launcher launcher;
    public final RobotSnapshot snapshot;
    public final PollenAssist assist;

    // Null until startMatch() is called. Nothing is counted down during init.
    private TimerEx matchTimer;

    // One object per state, made once. States are compared with == below, so
    // there must never be a second copy of any of them.
    private final IdleState idle = new IdleState();
    private final CollectingState collecting = new CollectingState();
    private final LaunchingState launching = new LaunchingState();
    private final AssistState assistState = new AssistState();
    private TopState topState;

    // Logging only.
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final VoltageSensor battery;   // null when the hub reports none

    // ---- Constructor ----

    // No logging in here: the constructor runs once during init, outside the loop.
    public Robot(HardwareMap hardwareMap, Pose startPose) {
        hMap = hardwareMap;
        driveTrain = new Drivetrain(this,startPose);
        assist = new PollenAssist(this);
        intake = new Intake(this);
        launcher = new Launcher(this);
        // NOTE: this snapshot is taken once, here, and is not refreshed later.
        // Its values stay as they were when the robot was built.
        snapshot = new RobotSnapshot(driveTrain.mode(), intake.mode(), launcher.state(), launcher.getEnabled(), launcher.getFullAuto(), driveTrain.follower().pose().x(), driveTrain.follower().pose().y(), driveTrain.follower().pose().heading(), topState == null ? "null" : topState.name(), assist.arrived());
        // MANUAL bulk caching: one hub read per loop serves every encoder read in
        // that loop, which keeps loops short. The cache has to be cleared once per
        // loop or the readings never change. PsiKit's OpMode base does that clear.
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        // The periodic commands are scheduled here, once. This is why the OpMode
        // must call Scheduler.reset() BEFORE "new Robot": a reset afterwards would
        // throw these away and no subsystem would ever run.
        Scheduler.schedule(driveTrain.periodic(), intake.periodic(), launcher.periodic());

        topState = idle;
        topState.enter(this);

        battery = hardwareMap.voltageSensor.iterator().hasNext()
                ? hardwareMap.voltageSensor.iterator().next() : null;
    }

    // ---- Intent methods (what OpModes and autos call) ----

    public void run(Command command) {
        Scheduler.schedule(command);
    }

    public void startMatch(double seconds) {
        matchTimer = new TimerEx(seconds);
        matchTimer.start();
    }
    public double matchRemaining() {
        return matchTimer == null ? Double.POSITIVE_INFINITY : matchTimer.getRemaining();
    }

    // Idle <-> Collecting. While a shot is in progress we do not interrupt it:
    // the toggle only changes which state Launching goes back to afterwards.
    public void toggleIntake() {
        TopState current = (topState == launching) ? launching.returnTo() : topState;
        TopState next = (current == collecting) ? idle : collecting;
        if (topState == launching) launching.setReturnTo(next);
        else setState(next);
    }

    public void toggleLauncher() {
        run(launcher.toggle());
    }

    public void launch() {
        run(launcher.launch());
        enterLaunching();
    }

    // Called every loop with the trigger state (a level, not a press), so
    // releasing the trigger is seen on the very next loop.
    public void setFullAuto(boolean on) {
        launcher.setFullAuto(on);
        if (on) enterLaunching();
    }

    // Guard: ignore the button if we are already assisting (would restart the
    // path) or launching (a shot is in progress and should finish first).
    public void startAssist() {
        if (topState == assistState || topState == launching) return;
        assistState.setReturnTo(topState);
        setState(assistState);
    }

    // Guard: only leave Assisting if we are in it. The button release also
    // arrives when the assist never started or has already timed out.
    public void endAssist() {
        if (topState != assistState) return;
        setState(assistState.ReturnTo());
    }

    public void toggleFieldCentric() {
        driveTrain.toggleFieldCentric();
    }

    public void unjam() {
        run(intake.unjam());
    }

    // ---- Lifecycle (called by RobotOpMode) ----

    public void initLoop() {}
    public void start() {}

    // Runs once per loop, right after Scheduler.execute(). The subsystems have
    // already acted this loop, so the top state decides using fresh results.
    public void afterScheduler() {
        Logger.recordOutput("Robot/TopState", topState.name());
        if (matchTimer != null) Logger.recordOutput("Robot/MatchRemainingSec", matchTimer.getRemaining());
        topState.execute(this);

        // Requested vs actual, side by side: TopState is what the robot is trying
        // to do, the lines below are what each subsystem is really doing.
        Logger.recordOutput("Robot/DriveMode", driveTrain.mode());
        Logger.recordOutput("Robot/IntakeMode", intake.mode());
        Logger.recordOutput("Robot/ShooterState", launcher.state());
        // ReadyToShoot is read live from the launcher. SnapshotReadyToLaunch is
        // the value the OpModes and autos read. Comparing the two in a log shows
        // whether the snapshot is keeping up.
        Logger.recordOutput("Robot/ReadyToShoot", launcher.state() == Launcher.State.READY);
        Logger.recordOutput("Robot/SnapshotReadyToLaunch", snapshot.readyToLaunch());

        // Time since this line ran last loop. The first value also includes the
        // wait in init, so ignore it.
        Logger.recordOutput("Robot/LoopTimeMs", loopTimer.milliseconds());
        loopTimer.reset();

        // A sagging battery explains a slow flywheel better than any PID term.
        // This read is one extra hub command per loop.
        if (battery != null) Logger.recordOutput("Robot/BatteryVolts", battery.getVoltage());

        assist.log();
    }

    // Leave nothing powered when the OpMode ends. assist.stop() is also what
    // stops the follower.
    public void stop() {
        intake.stop();
        launcher.stop();
        assist.stop();
    }

    // Guard: sticks are ignored during an assist. A manual request would flip
    // the drivetrain to MANUAL and fight the path being followed.
    public void drive(double forward, double strafe, double turn) {
        if (topState == assistState) return;
        driveTrain.drive(forward, strafe, turn);
    }

    // ---- State machine ----

    public RobotSnapshot robotSnapshot() {
        return snapshot;
    }

    public TopState topState() { return topState; }

    // The only place the top state changes. Always exit the old state before
    // entering the new one, so each state can clean up what it switched on.
    public void setState(TopState next) {
        if (next == topState) return;
        topState.exit(this);
        topState = next;
        topState.enter(this);
    }

    // Guard: already launching (keep the first return state), or assisting
    // (the assist owns the drivetrain, so no shot until it ends).
    private void enterLaunching() {
        if (topState == launching || topState == assistState) return;
        launching.setReturnTo(topState);
        setState(launching);
    }

    // ---- Helpers ----

    public void clearBulkCache() {
        LynxUtil.clearBulkCache(hMap);
    }

    public RobotSnapshot snapshot() {
        return snapshot;
    }
}
