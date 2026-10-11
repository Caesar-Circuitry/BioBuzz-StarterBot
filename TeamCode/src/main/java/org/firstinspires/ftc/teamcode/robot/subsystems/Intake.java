package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.conditional;
import static com.pedropathing.ivy.commands.Commands.infinite;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.constants.IntakeConstants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.psilynx.psikit.core.Logger;

public class Intake {
    public enum MODE {
        OFF,
        ON,
        REVERSE
    }
    private MODE mode = MODE.OFF;

    private final DcMotorEx motor;
    private final CRServo leftServo;
    private final CRServo rightServo;

    public Intake(Robot r) {
        motor = r.hMap.get(DcMotorEx.class, (IntakeConstants.MOTOR_NAME));
        leftServo = r.hMap.get(CRServo.class, (IntakeConstants.LEFT_SERVO_NAME));
        rightServo = r.hMap.get(CRServo.class, (IntakeConstants.RIGHT_SERVO_NAME));

        if (IntakeConstants.LEFT_SERVO_REVERSED) leftServo.setDirection(DcMotorSimple.Direction.REVERSE);
        if (IntakeConstants.RIGHT_SERVO_REVERSED) rightServo.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public  MODE mode () {return mode;}

    public Command on() {
        return instant(() -> mode = MODE.ON).requiring(motor);
    }
    public Command off() {
        return instant(() -> mode = MODE.OFF).requiring(motor);
    }
    public Command reverse() {
        return instant(() -> mode = MODE.REVERSE).requiring(motor);
    }

    public Command unjam() {
        return reverse().then(waitMs(IntakeConstants.UNJAM_MS)).then(on());
    }

    public Command toggle() {
        return conditional(() -> mode == MODE.OFF, on(), off());
    }

    public Command periodic() {
        return infinite(()-> {
            double power = powerFOR(mode);
            motor.setPower(power);
            leftServo.setPower(power);
            rightServo.setPower(power);

            Logger.recordOutput("Intake/Mode", mode);
            Logger.recordOutput("Intake/RequestedPower",power);
            Logger.recordOutput("Intake/CurrentMilliamps",motor.getCurrent(CurrentUnit.MILLIAMPS));
        });
    }

    public void stop() {
        motor.setPower(0.0);
        leftServo.setPower(0.0);
        rightServo.setPower(0.0);
        mode = MODE.OFF;
    }

    public static double powerFOR(MODE m){
        switch (m) {
            case ON: return IntakeConstants.ON_POWER;
            case REVERSE: return IntakeConstants.REVERSE_POWER;
            default: return IntakeConstants.OFF_POWER;
        }
    }

}
