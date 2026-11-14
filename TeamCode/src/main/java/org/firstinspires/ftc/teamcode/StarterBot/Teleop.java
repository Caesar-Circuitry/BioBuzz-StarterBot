package org.firstinspires.ftc.teamcode.StarterBot;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

@TeleOp
public class Teleop extends LinearOpMode {
    private static final double HOLD_TOLERANCE_INCHES = 2.0;
    private static final double HOLD_TOLERANCE_DEGREES = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.create(hardwareMap);
        boolean pedroReady = (follower != null);

        drivetrain drive = pedroReady ? null : new drivetrain(hardwareMap);

        launcher launch = new launcher(hardwareMap);
        intake intakeSystem = new intake(hardwareMap);
        BallCounter balls = new BallCounter(hardwareMap);
        LimelightVision vision = new LimelightVision(hardwareMap);
        String lastStatus = "";
        PollenAssist assist = new PollenAssist();

        boolean rumbledThisRev = false;
        boolean wasFull = false;
        boolean wasLaunching = false;
        Pose holdPose = null;

        try {
            waitForStart();

            while (opModeIsActive()) {
                boolean launching = gamepad1.left_trigger > 0.5;
                boolean assisting = gamepad1.left_bumper && !launching;

                vision.setPipeline(launching ? LimelightVision.pipeline.APRILTAG
                        : LimelightVision.pipeline.HIVE);
                vision.update();

                // ===== DRIVE / HOLD =====
                boolean onSpot = true;

                if (launching && pedroReady) {
                    if (!wasLaunching) {
                        holdPose = follower.pose();
                        follower.hold(holdPose);
                    }
                    Pose now = follower.pose();
                    double distance = Math.hypot(now.x() - holdPose.x(), now.y() - holdPose.y());
                    double headingError = Math.toDegrees(Math.abs(
                            AngleUnit.normalizeRadians(now.heading() - holdPose.heading())));
                    onSpot = distance < HOLD_TOLERANCE_INCHES && headingError < HOLD_TOLERANCE_DEGREES;
                } else {
                    double forward = -gamepad1.left_stick_y;
                    double lateral = gamepad1.left_stick_x;
                    double turn = gamepad1.right_stick_x;

                    if (assisting) {
                        assist.update(vision);
                        forward = assist.getForward();
                        turn = assist.getTurn();
                    }
                    if (pedroReady) {
                        follower.manual(forward, lateral, -turn);
                    } else {
                        drive.drive(lateral, forward, turn);
                    }
                }
                wasLaunching = launching;
                if (pedroReady) {
                    follower.update();
                }

                if (launching) {
                    launch.rev(launcher.TARGET_VELOCITY);
                } else {
                    launch.stopFlywheel();
                }
                if (gamepad1.dpad_down) {
                    launch.stopFeeding();
                }
                launch.setFeedBlocked(!onSpot);
                launch.update();
                launch.feed(gamepad1.dpad_up, 1.0);

                boolean ready = launch.isReady();
                if (ready && !rumbledThisRev) {
                    gamepad1.rumble(300);
                    rumbledThisRev = true;
                }
                if (!launching) {
                    rumbledThisRev = false;
                }

                balls.update();
                if (launch.consumeShot()) {
                    balls.removeOne();
                }
                if (gamepad1.dpad_left) {
                    balls.reset();
                }
                boolean full = balls.isFull();
                if (full && !wasFull) {
                    gamepad1.rumbleBlips(2);
                }
                wasFull = full;

                intake.IntakeState intakeState;
                if (gamepad1.right_trigger > 0.5 || assisting) {
                    intakeState = intake.IntakeState.INTAKE;
                } else if (gamepad1.right_bumper) {
                    intakeState = intake.IntakeState.OUTTAKE;
                } else {
                    intakeState = intake.IntakeState.INTAKEIDLE;
                }
                intakeSystem.setIntakeState(intakeState);

                String status = full ? "FULL" : intakeState.name();
                if (!status.equals(lastStatus)) {
                    if (full) {
                        signalFull(gamepad2);
                    } else {
                        signalIntake(gamepad2, intakeState);
                    }
                    lastStatus = status;
                }

                telemetry.addData("Pedro", pedroReady ? follower.mode() : "not set up (using old drivetrain)");
                telemetry.addData("On spot", onSpot);
                telemetry.addData("Balls", balls.getCount() + " / " + BallCounter.MAX_BALLS);
                telemetry.addData("Sensor (cm)", "%.1f", balls.getDistanceCm());
                telemetry.addData("Camera", vision.getPipeline() + (vision.isReady() ? "" : " (moving)"));
                telemetry.addData("Gamepad 2 shows", status);
                telemetry.addData("Pollen seen", vision.hasTarget());
                telemetry.addData("tx / ty", "%.1f / %.1f", vision.getTx(), vision.getTy());
                telemetry.addData("Launcher ready", ready);
                telemetry.addData("Feeding", launch.isFeeding());
                telemetry.addData("Filtered vel", launch.getFilteredVelocity());
                telemetry.addData("Battery V", launch.getBatteryVoltage());
                telemetry.update();
            }
        } finally {
            if (drive != null) drive.stop();
            launch.stop();
            intakeSystem.setIntakeState(intake.IntakeState.STOPPED);
            vision.stop();
        }
    }

    private void signalFull(Gamepad pad) {
        pad.setLedColor(1, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
        pad.rumbleBlips(3);
    }

    private void signalIntake(Gamepad pad, intake.IntakeState state) {
        switch (state) {
            case INTAKE:
                pad.setLedColor(0, 1, 0, Gamepad.LED_DURATION_CONTINUOUS);
                pad.rumbleBlips(1);
                break;
            case OUTTAKE:
                pad.setLedColor(1, 0, 0, Gamepad.LED_DURATION_CONTINUOUS);
                pad.rumbleBlips(2);
                break;
            case INTAKEIDLE:
                pad.setLedColor(0, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
                pad.rumble(0.3, 0.3, 150);
                break;
            case STOPPED:
                pad.setLedColor(1, 1, 1, Gamepad.LED_DURATION_CONTINUOUS);
                pad.rumble(1.0, 1.0, 500);
                break;
        }
    }
}