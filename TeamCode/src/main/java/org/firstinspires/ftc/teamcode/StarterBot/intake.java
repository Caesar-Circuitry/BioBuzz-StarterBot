package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.psilynx.psikit.core.Logger;

public class intake {
    public final DcMotorEx intakeMotor;
    public final CRServo intakeServo1, intakeServo2;
    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        STOPPED
    }

    private IntakeState state = IntakeState.STOPPED;

    public intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, constants.INTAKE_MOTOR);
        intakeServo1 = hardwareMap.get(CRServo.class, constants.INTAKE_SERVO_1);
        intakeServo2 = hardwareMap.get(CRServo.class, constants.INTAKE_SERVO_2);
    }
    public void setIntakeState(IntakeState state) {
        this.state = state;
        switch (state) {
            case INTAKE:
                intakeMotor.setPower(constants.INTAKE_MOTOR_POWER);
                intakeServo1.setPower(constants.INTAKE_SERVO_POWER);
                intakeServo2.setPower(-constants.INTAKE_SERVO_POWER);
                break;
            case OUTTAKE:
                intakeMotor.setPower(-constants.INTAKE_MOTOR_POWER);
                intakeServo1.setPower(-constants.INTAKE_SERVO_POWER);
                intakeServo2.setPower(constants.INTAKE_SERVO_POWER);
                break;
            case STOPPED:
                intakeMotor.setPower(0.0);
                intakeServo1.setPower(0.0);
                intakeServo2.setPower(0.0);
                break;
        }
    }

    public IntakeState getState() {
        return state;
    }

    /** Intake/... : the mode, the powers we sent, and what the motor really did. */
    public void log() {
        Logger.recordOutput("Intake/Mode", state);
        Logger.recordOutput("Intake/MotorPower", intakeMotor.getPower());
        Logger.recordOutput("Intake/Servo1Power", intakeServo1.getPower());
        Logger.recordOutput("Intake/Servo2Power", intakeServo2.getPower());
        Logger.recordOutput("Intake/MotorVelocityTicksPerSec", intakeMotor.getVelocity());
        // A high current with the intake on usually means a jam.
        Logger.recordOutput("Intake/CurrentMilliamps", intakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
    }
}
