package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.pedropathing.ivy.Command;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Intake {
    public final DcMotorEx intakeMotor;
    public final CRServo intakeServo1, intakeServo2;
    public enum IntakeState {
        INTAKE,
        OUTTAKE,
        STOPPED
    }

    public Intake(Robot r) {
        intakeMotor = r.hMap.get(DcMotorEx.class, "intake");
        intakeServo1 = r.hMap.get(CRServo.class, "intakeServo1");
        intakeServo2 = r.hMap.get(CRServo.class, "intakeServo2");
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

    public Command Intake(){
        return Command.build()
                .setExecute(() -> setIntakeState(IntakeState.INTAKE));

    }

}
