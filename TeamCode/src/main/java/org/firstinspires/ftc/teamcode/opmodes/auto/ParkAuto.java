package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.constants.MatchConstants;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode;
import org.firstinspires.ftc.teamcode.robot.autoPaths.ParkRoutine;

@Autonomous(name = "Park Auto", group = "Auto")
public class ParkAuto extends RobotOpMode {
    @Override
    protected Pose startPose() {
        return FieldConstants.AUTO_START;
    }

    @Override
    protected String modeName() {
        return "Auto";
    }
    @Override protected void onModeStart() {
        robot.startMatch(MatchConstants.AUTO_SECONDS);
        robot.run(ParkRoutine.build(robot));
    }
}
