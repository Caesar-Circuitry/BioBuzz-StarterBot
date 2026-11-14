package org.firstinspires.ftc.teamcode.StarterBot;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Sensor Test", group = "Tuning")
public class SensorTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        BallCounter balls = new BallCounter(hardwareMap);
        LimelightVision vision = new LimelightVision(hardwareMap);
        PollenAssist assist = new PollenAssist();
        HiveAim aim = new HiveAim();
        boolean red = true;
        boolean lastY = false;
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

        boolean lastA = false;
        boolean lastX = false;
        boolean tagMode = false;

        telemetry.addLine("A = pretend a shot, B = reset count, X = flip camera, Y = switch alliance");
        telemetry.update();
        waitForStart();

        while (opModeIsActive()) {
            balls.update();

            if (gamepad1.a && !lastA) {
                balls.removeOne();
            }
            lastA = gamepad1.a;

            if (gamepad1.b) {
                balls.reset();
            }

            if (gamepad1.x && !lastX) tagMode = !tagMode;
            lastX = gamepad1.x;
            vision.setPipeline(tagMode ? LimelightVision.pipeline.APRILTAG
                    : LimelightVision.pipeline.HIVE);
            vision.update();

            if (gamepad1.y && !lastY) red = !red;
            lastY = gamepad1.y;
            aim.update(vision, red, null);

            assist.update(vision);
            panels.addData("--- DISTANCE SENSOR ---", "");
            panels.addData("raw cm", balls.getRawDistanceCm());
            panels.addData("filtered cm", balls.getDistanceCm());
            panels.addData("ball present", balls.isBallPresent() ? 1 : 0);
            panels.addData("count", balls.getCount());
            panels.addData("full", balls.isFull());

            panels.addData("--- LIMELIGHT ---", "");
            panels.addData("camera mode", vision.getPipeline());
            panels.addData("camera ready", vision.isReady());
            panels.addData("tags seen", vision.getTagCount());
            panels.addData("sees RED hive", vision.seesHive(true));
            panels.addData("sees BLUE hive", vision.seesHive(false));

            panels.addData("--- HIVE AIM (camera up) ---", "");
            panels.addData("alliance (Y)", red ? "RED" : "BLUE");
            panels.addData("our tags used", aim.getTagsUsed());
            panels.addData("aim error deg (+ = left)", aim.getAimErrorDeg());
            panels.addData("hive distance in", aim.getDistanceIn());
            panels.addData("table speed", ShotTable.speedFor(aim.getDistanceIn(), launcher.TARGET_VELOCITY));
            panels.addData("table", ShotTable.status());
            panels.addData("pollen seen", vision.hasTarget());
            panels.addData("tx (+ = right)", vision.getTx());
            panels.addData("ty", vision.getTy());
            panels.addData("ta (size %)", vision.getTa());
            panels.addData("class", vision.getBestClassName());

            panels.addData("--- ASSIST WOULD DO ---", "");
            panels.addData("turn (+ = right)", assist.getTurn());
            panels.addData("forward", assist.getForward());

            panels.update(telemetry);
        }
        vision.stop();
    }
}