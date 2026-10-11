package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.psilynx.psikit.core.Logger;
import org.psilynx.psikit.ftc.PsiKitIterativeOpMode;

public abstract class RobotOpMode extends PsiKitIterativeOpMode {
    protected Robot robot;

   @Override protected int getRlogPort() {return 0;}

    @Override protected void onPsiKitConfigureLogging() {
        Logger.recordMetadata("Robot", "StarterBot");
        Logger.recordMetadata("Mode", modeName());
    }

    @Override protected void onPsiKitInit(){
        Scheduler.reset();
        robot = new Robot(hardwareMap, startPose());
    }

    @Override protected void onPsiKitInitLoop() {robot.initLoop();}

    @Override protected void onPsiKitStart() {
       robot.start();
       onModeStart();
   }

    @Override protected void onPsiKitLoop() {
        handleGamepads();
        Scheduler.execute();
        robot.afterScheduler();
        applyFeedback();
        Logger.recordOutput("Robot/Mode", modeName());
    }
    @Override protected void onPsiKitStop() {
       if (robot != null) {
           robot.stop();
       }
    }

    protected abstract Pose startPose();
    protected abstract String modeName();


    protected void handleGamepads() {}     // Blocks 3 and 5
    protected void applyFeedback() {}      // Block 5
    protected void onModeStart() {}        // Block 6

}
