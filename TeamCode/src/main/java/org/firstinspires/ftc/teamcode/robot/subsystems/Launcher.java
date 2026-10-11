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


public class Launcher {
    public enum State {
        OFF,
        SPINNING_UP,
        READY,
        FEEDING,
        RECOVERING
    }

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

    public Launcher(Robot r) {
        launcher = r.hMap.get(DcMotorEx.class, LauncherConstants.LAUNCHER_NAME);
        windmill = r.hMap.get(CRServo.class, LauncherConstants.WINDMILL_NAME);

        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher.setDirection(LauncherConstants.LAUNCHER_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);
        windmill.setDirection(LauncherConstants.WINDMILL_REVERSED ? DcMotor.Direction.REVERSE : DcMotor.Direction.FORWARD);

        pid = new PidController(LauncherConstants.KP, LauncherConstants.KI, LauncherConstants.KD, LauncherConstants.KF)
                .withIntegralLimit(LauncherConstants.I_LIMIT)
                .withOutputRange(0, LauncherConstants.MAX_POWER);
    }

    public State state() {
        return state;
    }

    public Command enable() {
        return instant(() -> launcherEnabled = true).requiring(launcher);
    }

    public Command disable() {
        return instant(() -> launcherEnabled = false).requiring(launcher);
    }

    public Command toggle() {
        return conditional(() -> launcherEnabled, disable(), enable());
    }

    public Command launch() {
        return instant(() -> shotRequested = true).requiring(launcher);
    }

    public void  setFullAuto(boolean on){fullAuto = on;}

    public Command periodic() {
        return infinite(() -> {
            double dt = loopTimer.seconds();
            loopTimer.reset();

            double target = launcherEnabled ? LauncherConstants.TARGET_SPEED : 0.0;
            double measured = launcher.getVelocity();
            double error = Math.abs(target - measured);
            boolean calibrated = target >0 && LauncherConstants.ENTER_TOLERANCE >0;
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
                     if (stateTimer.milliseconds() >= LauncherConstants.QUARTER_TURN_MS) {
                        goTo(State.RECOVERING);
                    }
                    break;
                case RECOVERING:
                    if (!launcherEnabled) {
                        goTo(State.OFF);
                    } else if (stateTimer.milliseconds() >= LauncherConstants.MIN_FEED_GAP_MS && inTight) {
                        goTo(State.READY);
                    }
                    break;
            }

            if (state != State.READY) shotRequested = false;

            double launcherPower = 0;
            if(state == State.OFF) pid.reset();
            else launcherPower = pid.update(target, measured, dt);

            double windmillPower = (state == State.FEEDING) ? LauncherConstants.WINDMILL_POWER : 0.0;
            launcher.setPower(launcherPower);
            windmill.setPower(windmillPower);

            Logger.recordOutput("Shooter/State", state);
            Logger.recordOutput("Shooter/LauncherEnabled", launcherEnabled);
            Logger.recordOutput("Shooter/FullAuto", fullAuto);
            Logger.recordOutput("Shooter/TargetSpeed", target);
            Logger.recordOutput("Shooter/MeasuredSpeed", measured);
            Logger.recordOutput("Shooter/Error", target - measured);
            Logger.recordOutput("Shooter/Pid/P", pid.pTerm());
            Logger.recordOutput("Shooter/Pid/I", pid.iTerm());
            Logger.recordOutput("Shooter/Pid/D", pid.dTerm());
            Logger.recordOutput("Shooter/Pid/F", pid.fTerm());
            Logger.recordOutput("Shooter/LauncherPower", launcherPower);
            Logger.recordOutput("Shooter/WindmillPower", windmillPower);
            Logger.recordOutput("Shooter/FeedCycles", feedCycles);
        });
    }

    public void stop() {
        launcher.setPower(0.0);
        windmill.setPower(0.0);
        state = State.OFF;
        launcherEnabled = false;
        shotRequested = false;
        fullAuto = false;
        feedCycles = 0;
    }
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