package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Teleop extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        drivetrain drive = new drivetrain(hardwareMap);
        launcher launch = new launcher(hardwareMap);
        intake intakeSystem = new intake(hardwareMap);

        boolean wasReady = false;

        try {
            waitForStart();

            while (opModeIsActive()) {
                // Drive control
                double x = gamepad1.left_stick_x;     // strafe (left stick)
                double y = -gamepad1.left_stick_y;    // forward/back (left stick)
                double rx = gamepad1.right_stick_x;   // turn (right stick)
                drive.drive(x, y, rx);

                if (gamepad1.right_trigger > 0.5) {
                    launch.rev(launcher.LAUNCH_POWER);
                    // launch.rev(launcher.LAUNCH_VELOCITY);
                } else {
                    launch.stopFlywheel();
                }
                launch.feed(gamepad1.a, 1.0);

                boolean ready = launch.isReady();
                if (ready && !wasReady) {
                    gamepad1.rumble(300);   // 300 ms buzz
                }
                wasReady = ready;

//                // Launcher control
//                if (gamepad1.a) {
//                    launch.launchSTANDARD();
//                } else if (gamepad1.b) {
//                    launch.launchVARIABLE(0.5, 1); // Example variable launch speed
//                } else {
//                    launch.stop();
//                }

                // Intake control
                intakeSystem.setIntakeState(intake.IntakeState.INTAKEIDLE);
                if (gamepad1.x) {
                    intakeSystem.setIntakeState(intake.IntakeState.INTAKE);
                } else if (gamepad1.y) {
                    intakeSystem.setIntakeState(intake.IntakeState.OUTTAKE);
                } else {
                    intakeSystem.setIntakeState(intake.IntakeState.STOPPED);
                }
                telemetry.addData("Launcher ready", ready);
                telemetry.addData("Flywheel velocity", launch.launcherMotor.getVelocity());
                telemetry.update();
            }
        } finally {
            drive.stop();
            launch.stop();
            intakeSystem.setIntakeState(intake.IntakeState.STOPPED);
        }
    }
}
