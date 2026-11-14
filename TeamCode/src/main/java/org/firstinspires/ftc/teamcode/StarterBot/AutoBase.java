package org.firstinspires.ftc.teamcode.StarterBot;

import static com.pedropathing.api.Paths.line;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.lazy;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.pedro.Constants;

@Configurable
public abstract class AutoBase extends LinearOpMode {
    public static int PRELOAD_BALLS = 3;
    public static double CAMERA_WAIT_SECONDS = 0.6;
    public static double SPINUP_TIMEOUT_SECONDS = 2.5;
    public static double SHOOT_TIMEOUT_SECONDS = 4.0;
    public static double SEARCH_TURN = 0.25;

    protected final PoseFactory p = PoseFactory.degrees();

    protected Follower follower;
    protected launcher launch;
    protected intake intakeSystem;
    protected BallCounter balls;
    protected LimelightVision vision;
    protected HiveAim aim;
    protected PollenAssist assist;

    protected boolean red = true;
    protected boolean launching = false;
    protected boolean feeding = false;
    protected intake.IntakeState intakeState = intake.IntakeState.INTAKEIDLE;
    private String step = "waiting";

    protected abstract Pose startPose();

    protected abstract Command routine();

    @Override
    public void runOpMode() throws InterruptedException {
        Scheduler.reset();

        follower = Constants.create(hardwareMap);
        if (follower == null) {
            telemetry.addLine("Pedro is not set up yet. Finish AutoTune and set Constants.TUNED = true.");
            telemetry.update();
            waitForStart();
            return;
        }

        launch = new launcher(hardwareMap);
        intakeSystem = new intake(hardwareMap);
        balls = new BallCounter(hardwareMap);
        vision = new LimelightVision(hardwareMap);
        aim = new HiveAim();
        assist = new PollenAssist();

        follower.setPose(startPose());
        follower.update();
        balls.setCount(PRELOAD_BALLS);

        while (opModeInInit()) {
            if (gamepad1.b) red = true;
            if (gamepad1.x) red = false;
            telemetry.addData("Alliance", red ? "RED  (press X for blue)" : "BLUE  (press B for red)");
            telemetry.addData("Preloaded balls", PRELOAD_BALLS);
            telemetry.addData("Distance table", ShotTable.status());
            telemetry.update();
        }
        vision.setAlliance(red);

        schedule(routine());

        try {
            while (opModeIsActive()) {
                vision.setPipeline(launching ? LimelightVision.pipeline.APRILTAG
                        : LimelightVision.pipeline.HIVE);
                vision.update();
                aim.update(vision, red, follower.pose());

                double speed = launcher.TARGET_VELOCITY;
                if (aim.hasAim()) {
                    speed = ShotTable.speedFor(aim.getDistanceIn(), launcher.TARGET_VELOCITY);
                    speed = Math.round(speed / 10.0) * 10.0;
                }
                if (launching) {
                    launch.rev(speed);
                } else {
                    launch.stopFlywheel();
                }
                launch.setFeeding(launching && feeding);
                launch.update();
                launch.feed(false, 1.0);

                balls.update();
                if (launch.consumeShot()) balls.removeOne();

                intakeSystem.setIntakeState(intakeState);

                follower.update();
                Scheduler.execute();

                Pose pose = follower.pose();
                telemetry.addData("Step", step);
                telemetry.addData("Pose", "x %.1f  y %.1f  h %.0f", pose.x(), pose.y(), Math.toDegrees(pose.heading()));
                telemetry.addData("Follower", follower.mode());
                telemetry.addData("Balls", balls.getCount());
                telemetry.addData("Aim", aim.getSource() + "  %.1f deg  %.1f in", aim.getAimErrorDeg(), aim.getDistanceIn());
                telemetry.addData("Flywheel target / actual", "%.0f / %.0f", speed, launch.getFilteredVelocity());
                telemetry.addData("Pollen seen", vision.hasTarget());
                telemetry.update();
            }
        } finally {
            launch.stop();
            intakeSystem.setIntakeState(intake.IntakeState.STOPPED);
            vision.stop();
        }
    }

    protected Command label(String name) {
        return instant(() -> step = name);
    }

    protected Command shootAll() {
        return sequential(
                label("shooting"),
                instant(() -> {
                    launching = true;
                    feeding = false;
                }),
                waitMs((int) (CAMERA_WAIT_SECONDS * 1000)),
                instant(this::aimAtHive),
                waitUntil(() -> launch.isReady()).raceWith(waitMs((int) (SPINUP_TIMEOUT_SECONDS * 1000))),
                instant(() -> feeding = true),
                waitUntil(() -> balls.getCount() == 0).raceWith(waitMs((int) (SHOOT_TIMEOUT_SECONDS * 1000))),
                waitMs(300),
                instant(() -> {
                    feeding = false;
                    launching = false;
                })
        );
    }

    protected Command driveTo(Pose target) {
        return lazy(() -> {
            Pose now = follower.pose();
            return follow(follower, line(now, target).linear(now, target));
        });
    }

    protected Command collectPollen(int target, double timeoutSeconds) {
        Command hunt = Command.build()
                .setStart(() -> {
                    step = "collecting pollen";
                    intakeState = intake.IntakeState.INTAKE;
                })
                .setExecute(() -> {
                    double forward = 0;
                    double turn = SEARCH_TURN;
                    if (vision.hasTarget()) {
                        assist.update(vision);
                        forward = assist.getForward();
                        turn = assist.getTurn();
                    }
                    follower.manual(forward, 0, -turn);
                })
                .setDone(() -> balls.getCount() >= target)
                .setEnd(endCondition -> {
                    follower.manual(0, 0, 0);
                    intakeState = intake.IntakeState.INTAKEIDLE;
                });
        return hunt.raceWith(waitMs((int) (timeoutSeconds * 1000)));
    }

    private void aimAtHive() {
        if (aim.getSource() != HiveAim.Source.TAGS) return;
        Pose now = follower.pose();
        double heading = Math.toDegrees(now.heading()) + aim.getAimErrorDeg();
        follower.hold(p.of(now.x(), now.y(), heading));
    }
}