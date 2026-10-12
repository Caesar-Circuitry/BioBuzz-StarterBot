package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.psilynx.psikit.core.Logger;

/**
 * The intake: one motor and two continuous servos that all spin together.
 *
 * Layer: subsystem (opmodes -> Robot -> subsystems). Commands here only change
 * the wanted mode. periodic() is the one place that writes power to hardware,
 * so the log always shows what was really sent.
 */
public class Intake {
    public enum MODE {
        OFF,
        ON,
        REVERSE
    }

    // ---- Fields ----

    private MODE mode = MODE.OFF;

    private final DcMotorEx motor;
    private final CRServo leftServo;
    private final CRServo rightServo;

    // ---- Constructor ----

    public Intake(Robot r) {
        motor = r.hMap.get(DcMotorEx.class, (IntakeConstants.MOTOR_NAME));
        leftServo = r.hMap.get(CRServo.class, (IntakeConstants.LEFT_SERVO_NAME));
        rightServo = r.hMap.get(CRServo.class, (IntakeConstants.RIGHT_SERVO_NAME));

        if (IntakeConstants.LEFT_SERVO_REVERSED) leftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        if (IntakeConstants.RIGHT_SERVO_REVERSED) rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    // ---- Commands ----

    public MODE mode() {return mode;}

    // Every intake command requires the motor. The scheduler lets only one
    // command hold a requirement, so a new intake command cancels the old one.
    // Example: pressing "off" in the middle of an unjam stops the unjam, so it
    // cannot switch the intake back on a moment later.
    public Command on() {
        return instant(() -> mode = MODE.ON).requiring(motor);
    }
    public Command off() {
        return instant(() -> mode = MODE.OFF).requiring(motor);
    }
    public Command reverse() {
        return instant(() -> mode = MODE.REVERSE).requiring(motor);
    }

    // Spit out for a moment, then go back to collecting.
    public Command unjam() {
        return reverse().then(waitMs(IntakeConstants.UNJAM_MS)).then(on());
    }

    public Command toggle() {
        return conditional(() -> mode == MODE.OFF, on(), off());
    }

    // ---- Periodic ----

    // Requires nothing, so the commands above (which require the motor) can
    // never cancel it. If this loop stopped, the intake would stop obeying.
    public Command periodic() {
        return infinite(() -> {
            double power = powerFOR(mode);
            motor.setPower(power);
            leftServo.setPower(power);
            rightServo.setPower(power);

            Logger.recordOutput("Intake/Mode", mode);
            // Requested power is what we asked for. Current is what the motor is
            // really doing: a high current with the intake on usually means a jam.
            Logger.recordOutput("Intake/RequestedPower", power);
            Logger.recordOutput("Intake/CurrentMilliamps", motor.getCurrent(CurrentUnit.MILLIAMPS));
        });
    }

    // ---- Helpers ----

    // Called at OpMode stop. The scheduler is no longer running then, so the
    // powers are written here directly and not left for periodic().
    public void stop() {
        motor.setPower(0.0);
        leftServo.setPower(0.0);
        rightServo.setPower(0.0);
        mode = MODE.OFF;
    }

    public static double powerFOR(MODE m) {
        switch (m) {
            case ON: return IntakeConstants.ON_POWER;
            case REVERSE: return IntakeConstants.REVERSE_POWER;
            default: return IntakeConstants.OFF_POWER;
        }
    }
}
