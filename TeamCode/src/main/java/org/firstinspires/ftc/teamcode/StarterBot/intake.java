package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class intake {
    public final DcMotorEx intakeMotor;
    public final CRServo intakeServo1, intakeServo2;
    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        STOPPED
    }

    public intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, "intake");
        intakeServo1 = hardwareMap.get(CRServo.class, "intakeServo1");
        intakeServo2 = hardwareMap.get(CRServo.class, "intakeServo2");
    }
    public void setIntakeState(IntakeState state) {
        switch (state) {
            case INTAKE:
                intakeMotor.setPower(1.0);
                intakeServo1.setPower(1.0);
                intakeServo2.setPower(-1.0);
                break;
            case OUTTAKE:
                intakeMotor.setPower(-1.0);
                intakeServo1.setPower(-1.0);
                intakeServo2.setPower(1.0);
                break;
            case STOPPED:
                intakeMotor.setPower(0.0);
                intakeServo1.setPower(0.0);
                intakeServo2.setPower(0.0);
                break;
        }
    }

}
