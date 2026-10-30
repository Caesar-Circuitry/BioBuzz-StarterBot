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

        try {
            waitForStart();

            while (opModeIsActive()) {
                // Drive control
                double x = gamepad1.left_stick_x;
                double y = -gamepad1.left_stick_y; // Invert Y axis
                double rx = gamepad1.right_stick_x;
                drive.drive(x, y, rx);

                // Launcher control
                if (gamepad1.a) {
                    launch.launchSTANDARD();
                } else if (gamepad1.b) {
                    launch.launchVARIABLE(0.5, 0.5); // Example variable launch speed
                } else {
                    launch.stop();
                }

                // Intake control
                if (gamepad1.x) {
                    intakeSystem.setIntakeState(intake.IntakeState.INTAKE);
                } else if (gamepad1.y) {
                    intakeSystem.setIntakeState(intake.IntakeState.OUTTAKE);
                } else {
                    intakeSystem.setIntakeState(intake.IntakeState.STOPPED);
                }
            }
        } finally {
            drive.stop();
            launch.stop();
            intakeSystem.setIntakeState(intake.IntakeState.STOPPED);
        }
    }
}
