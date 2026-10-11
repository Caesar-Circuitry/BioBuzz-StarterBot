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

public class PollenAssist {
    private Robot robot;
    private boolean following = false;
    private boolean arrived = false;
    private Pose target = null;

    public PollenAssist(Robot robot) {
        this.robot = robot;
        Weaver.setConfig(new PathConfig()
                .intakeWidth(AssistConstants.INTAKE_WIDTH)
                .robotWidth(AssistConstants.ROBOT_WIDTH)
                .robotHeight(AssistConstants.ROBOT_HEIGHT)
                .clearance(AssistConstants.CLEARANCE)
                .smoothing(AssistConstants.SMOOTHING));
    }

    public boolean start() {
        Pose here = robot.driveTrain.follower().pose();
        this.target = nearest(here);
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
        arrived = following && robot.driveTrain.follower().atParametricEnd();
        Logger.recordOutput("Assist/Active", following);
        Logger.recordOutput("Assist/Arrived", arrived);
        if (target !=null) {
            Logger.recordOutput("Assist/TargetXInches", target.x());
            Logger.recordOutput("Assist/TargetYInches", target.y());
        }
    }

    public boolean arrived() {return arrived;}

    public void stop() {
        robot.driveTrain.follower().stop();
        following =false;
        arrived =false;
        target = null;
    }

    private Pose nearest(Pose here) {
       Pose best = null;
         double bestDist = Double.MAX_VALUE;
         for (Pose p: FieldConstants.POLLEN_PRESETS) {
             double d = Math.hypot(p.x() - here.x(), p.y() - here.y());
             if (d < bestDist) {bestDist = d; best = p;}
         }
         return best;
    }

}
