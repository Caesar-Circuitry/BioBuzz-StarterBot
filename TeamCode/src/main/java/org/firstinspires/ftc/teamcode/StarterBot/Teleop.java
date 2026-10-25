package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Driver-controlled StarterBot, one gamepad. Same controls as before:
 *   left stick drive/strafe, right stick X turn
 *   A standard launch, B variable launch
 *   X intake, Y outtake
 *
 * Now an iterative PsiKit OpMode (see StarterBotOpMode), like the Cohen-Testing
 * branch, so every loop is logged. The old try/finally became onPsiKitStop().
 */
@TeleOp
public class Teleop extends StarterBotOpMode {
    private drivetrain drive;
    private launcher launch;
    private intake intakeSystem;

    @Override protected String modeName() {
        return "Teleop";
    }

    @Override protected void onPsiKitInit() {
        super.onPsiKitInit();
        drive = new drivetrain(hardwareMap);
        launch = new launcher(hardwareMap);
        intakeSystem = new intake(hardwareMap);
    }

    @Override protected void onPsiKitLoop() {
        // Drive control
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y; // Invert Y axis
        double rx = gamepad1.right_stick_x;
        drive.drive(x, rx, y);

        // Launcher control
        if (gamepad1.a) {
            launch.launchSTANDARD();
        } else if (gamepad1.b) {
            launch.launchVARIABLE(constants.LAUNCH_VARIABLE_FLYWHEEL_POWER, constants.LAUNCH_VARIABLE_WINDMILL_POWER);
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

        logControls();
        drive.log();
        launch.log();
        intakeSystem.log();
        logRobot();
    }

    // Leave nothing powered when the OpMode ends. Guard: stop can arrive when init
    // failed part way, before the subsystems exist.
    @Override protected void onPsiKitStop() {
        if (drive != null) drive.stop();
        if (launch != null) launch.stop();
        if (intakeSystem != null) intakeSystem.setIntakeState(intake.IntakeState.STOPPED);
    }
}
