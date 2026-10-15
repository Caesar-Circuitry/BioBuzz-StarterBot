package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class launcher {
    public final DcMotorEx launcherMotor;
    public final CRServo launcherServo;
    public launcher(HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherServo = hardwareMap.get(CRServo.class, "launcherServo");
    }
    public void launchVARIABLE(double launchSpeed, double ServoPower){
        launcherMotor.setPower(launchSpeed);
        launcherServo.setPower(ServoPower);
    }
    public void launchSTANDARD(){
        launcherMotor.setPower(1.0);
        launcherServo.setPower(1.0);//IDK what a 'standard' launch speed is
    }
    public void stop(){
        launcherMotor.setPower(0);
        launcherServo.setPower(0);
    }
}
