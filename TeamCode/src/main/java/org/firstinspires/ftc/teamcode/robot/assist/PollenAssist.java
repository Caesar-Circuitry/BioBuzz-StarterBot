package org.firstinspires.ftc.teamcode.robot.assist;

import com.pedropathing.math.Pose;
import com.skeletonarmy.marrow.weaver.PathConfig;
import com.skeletonarmy.marrow.weaver.PathResult;
import com.skeletonarmy.marrow.weaver.PedroPathingConverter;
import com.skeletonarmy.marrow.weaver.Weaver;

import org.firstinspires.ftc.teamcode.constants.AssistConstants;
import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.psilynx.psikit.core.Logger;

/**
 * Driver assist: while the button is held, drive the robot along a planned
 * path to the nearest known pollen spot.
 *
 * Layer: assist (opmodes -> Robot -> subsystems/assist/state). It borrows the
 * Drivetrain's follower. AssistState decides WHEN this runs; this class only
 * knows HOW to plan the path and follow it.
 */
public class PollenAssist {

    // The only place targets come from today is the preset list in
    // FieldConstants. Vision is not wired in yet.
    private static final String SOURCE = "PRESET";

    // ---- Fields ----

    private final Robot robot;
    private boolean following = false;
    private boolean arrived = false;
    private Pose target = null;

    // ---- Constructor ----

    public PollenAssist(Robot robot) {
        this.robot = robot;
        Weaver.setConfig(new PathConfig()
                .intakeWidth(AssistConstants.INTAKE_WIDTH)
                .robotWidth(AssistConstants.ROBOT_WIDTH)
                .robotHeight(AssistConstants.ROBOT_HEIGHT)
                .clearance(AssistConstants.CLEARANCE)
                .smoothing(AssistConstants.SMOOTHING));
    }

    // ---- Start, update, stop ----

    // Returns false when there is nowhere to go. The caller must then leave the
    // assist, or the robot would sit in "Assisting" with nothing driving it.
    public boolean start() {
        Pose here = robot.driveTrain.follower().pose();
        this.target = nearest(here);
        // Guard: no presets (the list starts empty) means no target.
        if (this.target == null) return false;

        PathResult result = Weaver.builder()
                .start(PedroPathingConverter.toPoint(here))
                .addTarget(PedroPathingConverter.toPoint(target))
                .maxTargets(1)
                .build();

        robot.driveTrain.follower().follow(PedroPathingConverter.toPath(result));
        following = true;
        arrived = false;
        return true;
    }

    public void update() {
        // Guard: "following &&" stops a finished path from an earlier assist (or
        // from auto) counting as "arrived" for this one.
        arrived = following && robot.driveTrain.follower().atParametricEnd();
    }

    // Called by Robot every loop, in every state, so the log also shows
    // Active = false when no assist is running. Only call from the OpMode loop.
    public void log() {
        Logger.recordOutput("Assist/Active", following);
        Logger.recordOutput("Assist/Arrived", arrived);
        Logger.recordOutput("Assist/Source", SOURCE);
        Logger.recordOutput("Assist/HasTarget", target != null);
        if (target != null) {
            Logger.recordOutput("Assist/TargetXInches", target.x());
            Logger.recordOutput("Assist/TargetYInches", target.y());
        }
    }

    public boolean arrived() {return arrived;}

    // Anything that starts the follower on a path must also stop it on the way
    // out. Without this the robot keeps driving after the button is released.
    public void stop() {
        // UNVERIFIED: Pedro has stop() here (there is no cancel()). Check on the
        // robot that it ends the path and lets the driver's sticks take over.
        robot.driveTrain.follower().stop();
        following = false;
        arrived = false;
        target = null;
    }

    // ---- Helpers ----

    // Straight-line distance only, which is enough to pick a target. The path
    // planner works out the real route afterwards.
    private Pose nearest(Pose here) {
        Pose best = null;
        double bestDist = Double.MAX_VALUE;
        for (Pose p : FieldConstants.POLLEN_PRESETS) {
            double d = Math.hypot(p.x() - here.x(), p.y() - here.y());
            if (d < bestDist) {bestDist = d; best = p;}
        }
        return best;
    }
}
