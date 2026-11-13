package org.firstinspires.ftc.teamcode.StarterBot;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;

// If Constants.create() isn't finished yet (returns null), this drives with the old
// drivetrain class and the hold-while-launching feature is turned off.
@TeleOp
public class Teleop extends LinearOpMode {
    private static final boolean RED_ALLIANCE = true;   // which nectar colour is ours

    // How far the robot can be pushed before feeding pauses
    private static final double HOLD_TOLERANCE_INCHES = 2.0;
    private static final double HOLD_TOLERANCE_DEGREES = 5.0;

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.create(hardwareMap);
        boolean pedroReady = (follower != null);

        // Backup drivetrain, only used while Pedro isn't set up
        drivetrain drive = pedroReady ? null : new drivetrain(hardwareMap);

        launcher launch = new launcher(hardwareMap);
        intake intakeSystem = new intake(hardwareMap);
        BallCounter balls = new BallCounter(hardwareMap);
        LimelightVision vision = new LimelightVision(hardwareMap);
        vision.setAlliance(RED_ALLIANCE);
        PollenAssist assist = new PollenAssist();

        boolean wasReady = false;
        boolean wasFull = false;
        boolean wasLaunching = false;
        Pose holdPose = null;

        try {
            waitForStart();

            while (opModeIsActive()) {
                boolean launching = gamepad1.left_trigger > 0.5;
                boolean assisting = gamepad1.left_bumper && !launching;

                // ===== CAMERA =====
                // Camera is UP on the AprilTags while the launcher is revving (left trigger) or
                // Y is held. The rest of the time, including intaking, it is DOWN on Hive vision.
                // X = go after pollen, B = go after our nectar.
                if (gamepad1.x) {
                    vision.setTarget(LimelightVision.target.POLLEN);
                } else if (gamepad1.b) {
                    vision.setTarget(LimelightVision.target.NECTAR);
                }
                if (launching || gamepad1.y) {
                    vision.setPipeline(LimelightVision.pipeline.APRILTAG);
                } else {
                    vision.setPipeline(LimelightVision.pipeline.HIVE);
                }
                vision.update();

                // ===== DRIVE / HOLD =====
                boolean onSpot = true;

                if (launching && pedroReady) {
                    if (!wasLaunching) {
                        // Just started launching: remember this spot and hold it
                        holdPose = follower.pose();
                        follower.hold(holdPose);
                    }
                    // How far have we been pushed?
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
                        follower.manual(forward, lateral, turn);   // switches Pedro back to driver control
                    } else {
                        drive.drive(lateral, forward, turn);        // old drivetrain: drive(x, y, rx)
                    }
                }
                wasLaunching = launching;
                if (pedroReady) {
                    follower.update();   // MUST run every loop when using Pedro
                }

                // ===== LAUNCHER =====
                // Hold left trigger to launch, D-pad up toggles feeding, D-pad down stops feeding
                if (launching) {
                    launch.rev(launcher.TARGET_VELOCITY);
                } else {
                    launch.stopFlywheel();
                }
                if (gamepad1.dpad_down) {
                    launch.stopFeeding();
                }
                launch.setFeedBlocked(!onSpot);   // pause feeding while pushed off the spot
                launch.update();
                launch.feed(gamepad1.dpad_up, 1.0);

                boolean ready = launch.isReady();
                if (ready && !wasReady) {
                    gamepad1.rumble(300);   // flywheel at speed
                }
                wasReady = ready;

                // ===== BALL COUNT =====
                balls.update();
                if (launch.consumeShot()) {
                    balls.removeOne();
                }
                if (gamepad1.dpad_left) {
                    balls.reset();   // fix the count by hand if it gets off
                }
                boolean full = balls.isFull();
                if (full && !wasFull) {
                    gamepad1.rumbleBlips(2);   // two buzzes = full
                }
                wasFull = full;

                // ===== INTAKE =====
                // Right trigger = intake, right bumper = outtake, pollen assist also intakes
                if (gamepad1.right_trigger > 0.5 || assisting) {
                    intakeSystem.setIntakeState(intake.IntakeState.INTAKE);
                } else if (gamepad1.right_bumper) {
                    intakeSystem.setIntakeState(intake.IntakeState.OUTTAKE);
                } else {
                    intakeSystem.setIntakeState(intake.IntakeState.INTAKEIDLE);
                }

                // ===== TELEMETRY =====
                telemetry.addData("Pedro", pedroReady ? follower.mode() : "not set up (using old drivetrain)");
                telemetry.addData("On spot", onSpot);
                telemetry.addData("Balls", balls.getCount() + " / " + BallCounter.MAX_BALLS);
                telemetry.addData("Sensor (cm)", "%.1f", balls.getDistanceCm());
                telemetry.addData("Camera", vision.getPipeline() + (vision.isReady() ? "" : " (switching)"));
                telemetry.addData("Looking for", vision.getTarget());
                telemetry.addData("Seen", vision.getBestClassName());
                telemetry.addData("Tags seen", vision.getTagCount());
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
}