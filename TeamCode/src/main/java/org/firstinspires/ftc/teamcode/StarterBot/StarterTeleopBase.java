package org.firstinspires.ftc.teamcode.StarterBot;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.pedro.Constants;

public abstract class StarterTeleopBase extends LinearOpMode {
    protected abstract boolean useDistanceSpeed();

    @Override
    public void runOpMode() throws InterruptedException {
        Follower follower = Constants.create(hardwareMap);
        boolean pedroReady = (follower != null);
        drivetrain drive = pedroReady ? null : new drivetrain(hardwareMap);

        launcher launch = new launcher(hardwareMap);
        intake intakeSystem = new intake(hardwareMap);
        BallCounter balls = new BallCounter(hardwareMap);
        LimelightVision vision = new LimelightVision(hardwareMap);
        PollenAssist assist = new PollenAssist();
        HiveAim aim = new HiveAim();

        boolean red = true;
        while (opModeInInit()) {
            if (gamepad1.b) red = true;
            if (gamepad1.x) red = false;
            telemetry.addData("Alliance", red ? "RED  (press X for blue)" : "BLUE  (press B for red)");
            telemetry.addData("Speed mode", useDistanceSpeed() ? "Distance table" : "Fixed");
            telemetry.addData("Distance table", ShotTable.status());
            telemetry.addData("Pedro", pedroReady ? "ready" : "not set up (no odometry aiming)");
            telemetry.update();
        }
        vision.setAlliance(red);

        boolean rumbledThisRev = false;
        boolean wasFull = false;
        String lastStatus = "";

        try {
            while (opModeIsActive()) {
                boolean launching = gamepad1.left_trigger > 0.5;
                boolean assisting = gamepad1.left_bumper && !launching;

                vision.setPipeline(launching ? LimelightVision.pipeline.APRILTAG
                        : LimelightVision.pipeline.HIVE);
                vision.update();
                Pose pose = pedroReady ? follower.pose() : null;
                aim.update(vision, red, pose);

                double forward = -gamepad1.left_stick_y;
                double lateral = gamepad1.left_stick_x;
                double turn = gamepad1.right_stick_x;

                if (assisting) {
                    assist.update(vision);
                    forward = assist.getForward();
                    turn = assist.getTurn();
                } else if (launching && aim.hasAim()) {
                    turn = aim.getTurn();
                }

                if (pedroReady) {
                    follower.manual(forward, lateral, -turn);
                    follower.update();
                } else {
                    drive.drive(lateral, forward, turn);
                }

                double speed = launcher.TARGET_VELOCITY;
                if (useDistanceSpeed() && aim.hasAim()) {
                    speed = ShotTable.speedFor(aim.getDistanceIn(), launcher.TARGET_VELOCITY);
                    speed = Math.round(speed / 10.0) * 10.0;
                }
                if (launching) {
                    launch.rev(speed);
                } else {
                    launch.stopFlywheel();
                }
                if (gamepad1.dpad_down) {
                    launch.stopFeeding();
                }

                launch.setFeedBlocked(aim.hasAim() && !aim.isAimed());
                launch.update();
                launch.feed(gamepad1.dpad_up, 1.0);

                boolean ready = launch.isReady();
                if (ready && !rumbledThisRev) {
                    gamepad1.rumble(300);
                    rumbledThisRev = true;
                }
                if (!launching) rumbledThisRev = false;

                balls.update();
                if (launch.consumeShot()) balls.removeOne();
                if (gamepad1.dpad_left) balls.reset();
                boolean full = balls.isFull();
                if (full && !wasFull) gamepad1.rumbleBlips(2);
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

                String status = (launching && aim.isAimed()) ? "AIMED"
                        : full ? "FULL"
                          : intakeState.name();
                if (!status.equals(lastStatus)) {
                    if (status.equals("AIMED")) {
                        signalAimed(gamepad2);
                    } else if (status.equals("FULL")) {
                        signalFull(gamepad2);
                    } else {
                        signalIntake(gamepad2, intakeState);
                    }
                    lastStatus = status;
                }

                telemetry.addData("Alliance", red ? "RED" : "BLUE");
                telemetry.addData("Aim source", aim.getSource() + "  (tags seen: " + aim.getTagsUsed() + ")");
                telemetry.addData("Aim error (deg)", "%.1f  %s", aim.getAimErrorDeg(), aim.isAimed() ? "AIMED" : "");
                telemetry.addData("Hive distance (in)", "%.1f", aim.getDistanceIn());
                telemetry.addData("Flywheel target / actual", "%.0f / %.0f", speed, launch.getFilteredVelocity());
                telemetry.addData("Ready / feeding", ready + " / " + launch.isFeeding());
                telemetry.addData("Pollen", balls.getCount() + " / " + BallCounter.MAX_BALLS);
                telemetry.addData("Camera", vision.getPipeline() + (vision.isReady() ? "" : " (moving)"));
                telemetry.addData("Gamepad 2 shows", status);
                telemetry.addData("Battery V", "%.2f", launch.getBatteryVoltage());
                telemetry.update();
            }
        } finally {
            if (drive != null) drive.stop();
            launch.stop();
            intakeSystem.setIntakeState(intake.IntakeState.STOPPED);
            vision.stop();
        }
    }

    private void signalAimed(Gamepad pad) {
        pad.setLedColor(0.6, 0, 1, Gamepad.LED_DURATION_CONTINUOUS);
        pad.rumble(1.0, 1.0, 250);
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