package org.firstinspires.ftc.teamcode.robot.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Robot;

public class Launcher {
    public final DcMotorEx launcherMotor;
    public final CRServo launcherServo;
    public Launcher(Robot r) {
        launcherMotor = r.hMap.get(DcMotorEx.class, "launcher");
        launcherServo = r.hMap.get(CRServo.class, "launcherServo");
    }
}
