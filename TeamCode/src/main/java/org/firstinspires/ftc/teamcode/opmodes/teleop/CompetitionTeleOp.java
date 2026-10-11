package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.FieldConstants;
import org.firstinspires.ftc.teamcode.opmodes.RobotOpMode;

@TeleOp(name = "Competition TeleOp", group = "TeleOp")
public class CompetitionTeleOp extends RobotOpMode {
    @Override protected Pose startPose() {
        return FieldConstants.TELEOP_START;
    }

    @Override protected String modeName() {
        return "Competition TeleOp";
    }
}
