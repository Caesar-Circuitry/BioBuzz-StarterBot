package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.constants.LauncherConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.control.PidController;
import org.psilynx.psikit.core.Logger;

/**
 * The shooter: a flywheel (the "launcher" motor) and a windmill servo that
 * feeds one pollen into the wheel per quarter turn. Logged under "Shooter/".
 *
 * Layer: subsystem (opmodes -> Robot -> subsystems). Commands and setters only
 * record what is wanted. periodic() runs the state machine and is the one
 * place that writes power to hardware.
 */
public class Launcher {

    // OFF         wheel not powered
    // SPINNING_UP wheel powered, not at speed yet
    // READY       at speed, waiting for a shot request
    // FEEDING     windmill turning, one pollen going into the wheel
    // RECOVERING  windmill stopped, wheel getting back the speed the shot took
    public enum State {
        OFF,
        SPINNING_UP,
        READY,
        FEEDING,
        RECOVERING
    }

    // ---- Fields ----

    private State state = State.OFF;
    private boolean launcherEnabled = false;
    private boolean shotRequested = false;
    private boolean fullAuto = false;
    private int feedCycles = 0;

    private final DcMotorEx launcher;
    private final CRServo windmill;
    private final PidController pid;
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime stateTimer = new ElapsedTime();

    // ---- Constructor ----

    public Launcher(Robot r) {
        launcher = r.hMap.get(DcMotorEx.class, LauncherConstants.LAUNCHER_NAME);
        windmill = r.hMap.get(CRServo.class, LauncherConstants.WINDMILL_NAME);

        // RUN_WITHOUT_ENCODER switches off the hub's built-in speed control, because
        // our own PID sets the power. The encoder is still read for getVelocity().
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // FLOAT lets the wheel coast. Braking a heavy flywheel is hard on the motor.
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setDirection(LauncherConstants.LAUNCHER_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        windmill.setDirection(LauncherConstants.WINDMILL_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        // The output range starts at 0, so the PID can only push the wheel forward.
        // If the wheel is too fast it coasts down; it is never driven backwards.
        pid = new PidController(LauncherConstants.KP, LauncherConstants.KI, LauncherConstants.KD, LauncherConstants.KF)
                .withIntegralLimit(LauncherConstants.I_LIMIT)
                .withOutputRange(0, LauncherConstants.MAX_POWER);
    }

    // ---- Commands ----

    public State state() {
        return state;
    }

    // All launcher commands require the motor, so only one can run at a time.
    public Command enable() {
        return instant(() -> launcherEnabled = true).requiring(launcher);
    }

    public Command disable() {
        return instant(() -> launcherEnabled = false).requiring(launcher);
    }

    public Command toggle() {
        return conditional(() -> launcherEnabled, disable(), enable());
    }

    // Single shot: a one-time request. periodic() uses it up, or drops it.
    public Command launch() {
        return instant(() -> shotRequested = true).requiring(launcher);
    }

    // Full auto is a level, not a request: the OpMode sends the trigger state
    // every loop, so letting go of the trigger stops the feeding straight away.
    public void setFullAuto(boolean on) {fullAuto = on;}

    // ---- Periodic ----

    // Requires nothing, so the commands above (which require the motor) can
    // never cancel it. The state machine must keep running all match.
    public Command periodic() {
        return infinite(() -> {
            // Real time since the last loop. Loop time is not constant, and the
            // PID's I and D terms are wrong if they assume it is.
            double dt = loopTimer.seconds();
            loopTimer.reset();

            // Logging only: remember the request before the state machine clears it.
            boolean shotRequestedThisLoop = shotRequested;

            double target = launcherEnabled ? LauncherConstants.TARGET_SPEED : 0.0;
            double measured = launcher.getVelocity();
            double error = Math.abs(target - measured);
            // The "uncalibrated guard". The constants start at 0 until someone
            // measures them. With a target of 0 and a tolerance of 0, a wheel that
            // is standing still has an error of 0, and 0 <= 0 would count as "at
            // speed". The robot would then feed pollen into a wheel that is not
            // spinning. So "in tolerance" only counts once both numbers are real.
            boolean calibrated = target > 0 && LauncherConstants.ENTER_TOLERANCE > 0;
            boolean inTight = calibrated && error <= LauncherConstants.ENTER_TOLERANCE;

            switch (state) {
                case OFF:
                    if (launcherEnabled) {
                        goTo(State.SPINNING_UP);
                    }
                    break;
                case SPINNING_UP:
                    if (!launcherEnabled) {
                        goTo(State.OFF);
                    } else if (inTight) {
                        goTo(State.READY);
                    }
                    break;
                case READY:
                    // Leaving READY has its own tolerance (LEAVE), meant to be wider
                    // than the one for entering. With one number, a speed sitting
                    // right on the edge would flip between READY and SPINNING_UP
                    // every loop.
                    if (!launcherEnabled) {
                        goTo(State.OFF);
                    } else if (calibrated && error > LauncherConstants.LEAVE_TOLERANCE) {
                        goTo(State.SPINNING_UP);
                    } else if (shotRequested || fullAuto) {
                        shotRequested = false;
                        goTo(State.FEEDING);
                    }
                    break;
                case FEEDING:
                    // Timed, because the windmill has no sensor. There is no
                    // "disabled" exit here: a feed that has started always finishes
                    // its turn, and RECOVERING handles the switch-off.
                    if (stateTimer.milliseconds() >= LauncherConstants.QUARTER_TURN_MS) {
                        goTo(State.RECOVERING);
                    }
                    break;
                case RECOVERING:
                    // Going back to READY needs two things: a minimum gap, so two
                    // pollen cannot touch the wheel together, AND the wheel back at
                    // speed after the last shot.
                    if (!launcherEnabled) {
                        goTo(State.OFF);
                    } else if (stateTimer.milliseconds() >= LauncherConstants.MIN_FEED_GAP_MS && inTight) {
                        goTo(State.READY);
                    }
                    break;
            }

            // A shot asked for while not READY is dropped, not saved. Otherwise a
            // press during spin-up would fire by surprise seconds later.
            if (state != State.READY) shotRequested = false;

            double launcherPower = 0;
            // Reset while OFF so integral built up in the last run is gone before
            // the next spin-up.
            if (state == State.OFF) pid.reset();
            else launcherPower = pid.update(target, measured, dt);

            double windmillPower = (state == State.FEEDING) ? LauncherConstants.WINDMILL_POWER : 0.0;
            launcher.setPower(launcherPower);
            windmill.setPower(windmillPower);

            Logger.recordOutput("Shooter/State", state);
            Logger.recordOutput("Shooter/LauncherEnabled", launcherEnabled);
            Logger.recordOutput("Shooter/FullAuto", fullAuto);
            Logger.recordOutput("Shooter/ShotRequested", shotRequestedThisLoop);
            Logger.recordOutput("Shooter/Calibrated", calibrated);
            Logger.recordOutput("Shooter/TimeInStateMs", stateTimer.milliseconds());
            // Target is what we asked for, Measured is what happened. The error is
            // signed here (positive = too slow) even though the logic uses its size.
            Logger.recordOutput("Shooter/TargetSpeedTicksPerSec", target);
            Logger.recordOutput("Shooter/MeasuredSpeedTicksPerSec", measured);
            Logger.recordOutput("Shooter/ErrorTicksPerSec", target - measured);
            Logger.recordOutput("Shooter/Pid/P", pid.pTerm());
            Logger.recordOutput("Shooter/Pid/I", pid.iTerm());
            Logger.recordOutput("Shooter/Pid/D", pid.dTerm());
            Logger.recordOutput("Shooter/Pid/F", pid.fTerm());
            Logger.recordOutput("Shooter/LauncherPower", launcherPower);
            Logger.recordOutput("Shooter/WindmillPower", windmillPower);
            Logger.recordOutput("Shooter/FeedCycles", feedCycles);
        });
    }

    // ---- Helpers ----

    // Called at OpMode stop. The scheduler is no longer running then, so the
    // powers are written here directly and not left for periodic().
    public void stop() {
        launcher.setPower(0.0);
        windmill.setPower(0.0);
        state = State.OFF;
        launcherEnabled = false;
        shotRequested = false;
        fullAuto = false;
        feedCycles = 0;
    }

    // Every state change goes through here so the state timer always restarts.
    private void goTo(State next) {
        state = next;
        stateTimer.reset();
        if (next == State.FEEDING) {
            feedCycles++;
        }
    }

    public boolean getEnabled() {
        return launcherEnabled;
    }

    public boolean getFullAuto() {
        return fullAuto;
    }
}