package org.firstinspires.ftc.teamcode.StarterBot;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Configurable
@TeleOp(name = "Launcher Tuner", group = "Tuning")
public class LauncherTuner extends LinearOpMode {
    public static boolean RUN_FLYWHEEL = false;

    @Override
    public void runOpMode() throws InterruptedException {
        launcher launch = new launcher(hardwareMap);
        TelemetryManager panels = PanelsTelemetry.INSTANCE.getTelemetry();

        boolean spinning = false;
        boolean lastA = false;

        waitForStart();

        while (opModeIsActive()) {
            if (gamepad1.a && !lastA) spinning = !spinning;
            lastA = gamepad1.a;

            if (spinning || RUN_FLYWHEEL) {
                launch.rev(launcher.TARGET_VELOCITY);
            } else {
                launch.stopFlywheel();
            }
            launch.update();

            launch.feed(gamepad1.right_bumper, 1.0);

            panels.addData("target", launch.getTargetVelocity());
            panels.addData("raw", launch.getRawVelocity());
            panels.addData("filtered", launch.getFilteredVelocity());
            panels.addData("power", launch.getMotorPower());
            panels.addData("battery", launch.getBatteryVoltage());
            panels.addData("ready", launch.isReady() ? 1 : 0);
            panels.update(telemetry);
        }
        launch.stop();
    }
}
