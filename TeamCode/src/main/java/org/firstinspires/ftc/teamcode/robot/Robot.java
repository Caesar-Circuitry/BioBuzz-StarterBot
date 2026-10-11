package org.firstinspires.ftc.teamcode.robot;

import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.skeletonarmy.marrow.LynxUtil;

import org.firstinspires.ftc.teamcode.robot.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.robot.subsystems.Intake;
import org.firstinspires.ftc.teamcode.robot.subsystems.Launcher;

public class Robot {
    public final HardwareMap hMap;
    public final Drivetrain driveTrain;
    public final Intake intake;
    public final Launcher launcher;

    public Robot(HardwareMap hardwareMap, Pose startPose) {
        hMap = hardwareMap;
        driveTrain = new Drivetrain(this,startPose);
        intake = new Intake(this);
        launcher = new Launcher(this);
        LynxUtil.setBulkCachingMode(hardwareMap, LynxModule.BulkCachingMode.MANUAL);

        Scheduler.schedule(driveTrain.periodic(), intake.periodic(), launcher.periodic());
    }

    public void run(Command command) {
        Scheduler.schedule(command);
    }

    public void toggleIntake(){
        run(intake.toggle());
    }

    public void toggleLauncher(){
        run(launcher.toggle());
    }
    public void launch(){
        run(launcher.launch());
    }
    public void setFullAuto(boolean on){
        launcher.setFullAuto(on);
    }

    public void toggleFieldCentric(){
        driveTrain.toggleFieldCentric();
    }
    public void unjam(){
        run(intake.unjam());
    }

    public void initLoop() {}
    public void start() {}
    public void afterScheduler() {}
    public void stop() {
        intake.stop();
        launcher.stop();
    }

    public void drive(double forward, double strafe, double turn) {
        driveTrain.drive(forward, strafe, turn);
    }


    public void clearBulkCache() {
        LynxUtil.clearBulkCache(hMap);
    }
}
