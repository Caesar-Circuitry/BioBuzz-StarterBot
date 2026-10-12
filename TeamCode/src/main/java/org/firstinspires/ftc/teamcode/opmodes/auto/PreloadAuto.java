package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.MatchConstants;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.autoPaths.ParkRoutine;
import org.firstinspires.ftc.teamcode.robot.autoPaths.PreloadRoutine;
import org.firstinspires.ftc.teamcode.robot.state.AutoHandoff;
import org.psilynx.psikit.core.Logger;

@Autonomous(name = "Preload Auto", group = "Auto")
/**
 * Autonomous OpMode: shoot the preloaded pollen, then park.
 *
 * Layer: opmodes. It only picks a routine and starts the clock; the routine
 * itself lives in robot.autoPaths and the loop order lives in RobotOpMode.
 */
public class PreloadAuto extends RobotOpMode {
    @Override
    protected Pose startPose() {
        return FieldConstants.AUTO_START;
    }

    @Override
    protected String modeName() {
        return "Preload Auto";
    }

    // clear() first, so a pose saved by an earlier auto run cannot leak into
    // the TeleOp that follows this one.
    @Override protected void onModeStart() {
        AutoHandoff.clear();
        robot.startMatch(MatchConstants.AUTO_SECONDS);
        robot.run(PreloadRoutine.build(robot));
    }

    // This is the per-loop hook that runs inside the logger. The routine has
    // no stages it reports, so only its name is logged.
    @Override protected void applyFeedback() {
        Logger.recordOutput("Autos/Routine", PreloadRoutine.NAME);
    }

    // Save the pose BEFORE super stops the robot, so TeleOp can start from
    // where auto really ended and not from a typed-in start pose.
    @Override protected void onPsiKitStop() {
        AutoHandoff.save(robot.driveTrain.follower().pose());
        super.onPsiKitStop();
    }
}
