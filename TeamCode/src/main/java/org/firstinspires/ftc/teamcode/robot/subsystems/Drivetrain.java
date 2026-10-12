package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.constants.ControlConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.logging.PedroLogger;
import org.psilynx.psikit.core.Logger;

/**
 * The drive base. Wraps the Pedro follower, which owns the four drive motors
 * and the localizer.
 *
 * Layer: subsystem (opmodes -> Robot -> subsystems). Only Robot, the assist and
 * the auto routines talk to this class. It never imports robot.state.
 */
public class Drivetrain {

    // PATH:   the follower is running a path or a hold (autos, pollen assist).
    //         periodic() must NOT write stick powers, or it would fight the path.
    // MANUAL: the driver's sticks are sent to the motors every loop.
    public enum Mode {
        PATH,
        MANUAL
    }

    // ---- Fields ----

    private final Follower follower;
    // Starts in PATH so an auto can follow a path without anyone asking first.
    // TeleOp switches to MANUAL on its first drive() call.
    private Mode mode = Mode.PATH;
    // The last stick request. Kept as fields because the request arrives in
    // handleGamepads() but is applied later, inside periodic().
    private double forward, strafe, turn;
    private boolean fieldCentric = true;

    // ---- Constructor ----

    public Drivetrain(Robot r, Pose startPose) {
        follower = Constants.create(r.hMap).withLogger(PedroLogger::logFollower);
        follower.setPose(startPose);
        follower.update();
    }

    // ---- Requests (called by Robot) ----

    public Follower follower() {return follower;}

    // Any stick request takes the robot out of PATH mode. That is why Robot
    // blocks drive() during an assist: one call here would cancel the path's
    // ownership of the motors.
    public void drive(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        mode = Mode.MANUAL;
    }

    public void setFieldCentric(boolean on) {fieldCentric = on;}
    public void usePathMode() {mode = Mode.PATH;}

    // ---- Periodic ----

    // Runs every loop for the whole OpMode. It declares no requirements on
    // purpose: a command that requires something can be cancelled by another
    // command that requires the same thing, and this loop must never be cancelled.
    public Command periodic() {
        return infinite(() -> {
            if (mode == Mode.MANUAL) {
                if (fieldCentric) {
                    DrivePowers p = ManualDrive.fieldCentric(forward, strafe, turn,
                            follower.pose().heading() + ControlConstants.FIELD_HEADING_OFFSET_RAD);
                    ManualDrive.driveOrHold(follower, p);
                } else {
                    ManualDrive.driveOrHold(follower, forward, strafe, turn);
                }
            }
            // update() runs in BOTH modes: it reads the localizer (so the pose stays
            // fresh while driving by hand) and, in PATH mode, steers along the path.
            follower.update();
            PedroLogger.logPose(follower.pose());

            // Requested values. During an assist Robot.drive() is blocked, so these
            // keep the last manual request and Mode shows PATH: the sticks are ignored.
            Logger.recordOutput("Drive/Mode", mode);
            Logger.recordOutput("Drive/FieldCentric", fieldCentric);
            Logger.recordOutput("Drive/RequestedForward", forward);
            Logger.recordOutput("Drive/RequestedStrafe", strafe);
            Logger.recordOutput("Drive/RequestedTurn", turn);
        });
    }

    // ---- Helpers ----

    public Mode mode() {
        return mode;
    }

    public void toggleFieldCentric() {
        fieldCentric = !fieldCentric;
    }
}
