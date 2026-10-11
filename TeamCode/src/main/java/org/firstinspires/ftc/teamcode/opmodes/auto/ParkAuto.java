package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.MatchConstants;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.autoPaths.ParkRoutine;
import org.firstinspires.ftc.teamcode.robot.state.AutoHandoff;

@Autonomous(name = "Park Auto", group = "Auto")
public class ParkAuto extends RobotOpMode {
    @Override
    protected Pose startPose() {
        return FieldConstants.AUTO_START;
    }

    @Override
    protected String modeName() {
        return "Park Auto";
    }
    @Override protected void onModeStart() {
        AutoHandoff.clear();
        robot.startMatch(MatchConstants.AUTO_SECONDS);
        robot.run(ParkRoutine.build(robot));
    }
    @Override protected void onPsiKitStop(){
        AutoHandoff.save(robot.driveTrain.follower().pose());
        super.onPsiKitStop();
    }
}
