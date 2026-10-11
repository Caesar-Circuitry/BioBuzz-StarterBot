package org.firstinspires.ftc.teamcode.opmodes.teleop;

import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.constants.ControlConstants;
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

    @Override protected void handleGamepads() {
        double forward = deadband(-gamepad1.left_stick_y);
        double strafe = deadband(gamepad1.left_stick_x);
        double turn = deadband(gamepad1.right_stick_x);
        robot.drive(forward, strafe, turn);
    }

    private static double deadband(double v) {
        return Math.abs(v) < ControlConstants.STICK_DEADBAND ? 0 : v;
    }
}
