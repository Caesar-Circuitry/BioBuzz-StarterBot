package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Objects;

public class intake {
    private final DcMotorEx intakeMotor;

    public enum IntakeState {
        INTAKE,
        INTAKEIDLE,
        OUTTAKE,
        STOPPED
    }

    public intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
    }

    public void setIntakeState(IntakeState state) {
        Objects.requireNonNull(state);
        switch (state) {
            case INTAKE:
                intakeMotor.setPower(1.0);
                break;
            case INTAKEIDLE:
                intakeMotor.setPower(0.3);
                break;
            case OUTTAKE:
                intakeMotor.setPower(-1.0);
                break;
            case STOPPED:
                intakeMotor.setPower(0.0);
                break;
        }
    }

    public void stop() {
        setIntakeState(IntakeState.STOPPED);
    }
}
