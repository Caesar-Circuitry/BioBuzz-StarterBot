package org.firstinspires.ftc.teamcode.StarterBot;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

@Configurable
@TeleOp(name = "Camera Servo Tuner", group = "Tuning")
public class CameraServoTuner extends LinearOpMode {
    public static double POSITION = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {
        Servo mount = hardwareMap.get(Servo.class, "LLMount");
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

        boolean lastUp = false, lastDown = false, lastRB = false, lastLB = false;

        telemetry.addLine("Press Start. The servo moves to the current HIVE position first.");
        telemetry.update();
        waitForStart();

        POSITION = LimelightVision.HivePosition;

        while (opModeIsActive()) {
            if (gamepad1.dpad_up && !lastUp)      POSITION += 0.01;
            if (gamepad1.dpad_down && !lastDown)  POSITION -= 0.01;
            if (gamepad1.right_bumper && !lastRB) POSITION += 0.05;
            if (gamepad1.left_bumper && !lastLB)  POSITION -= 0.05;
            lastUp = gamepad1.dpad_up;
            lastDown = gamepad1.dpad_down;
            lastRB = gamepad1.right_bumper;
            lastLB = gamepad1.left_bumper;

            POSITION = Range.clip(POSITION, 0.0, 1.0);
            mount.setPosition(POSITION);

            if (gamepad1.x) LimelightVision.HivePosition = POSITION;
            if (gamepad1.y) LimelightVision.AprilTagPosition = POSITION;
//
//            panels.addData("POSITION (now)", "%.2f", POSITION);
//            panels.addData("Saved HIVE (X)", "%.2f", LimelightVision.HivePosition);
//            panels.addData("Saved APRILTAG (Y)", "%.2f", LimelightVision.AprilTagPosition);
            panels.addData("Steps", "D-pad = 0.01, bumpers = 0.05");
            panels.addData("When done", "copy the saved numbers into LimelightVision.java");
            panels.update(telemetry);
        }
    }
}
