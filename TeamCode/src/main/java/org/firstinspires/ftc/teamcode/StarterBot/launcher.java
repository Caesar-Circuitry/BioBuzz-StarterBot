package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class launcher{
    private static final double revSeconds = 1.5;

    public final DcMotorEx launcherMotor;
    public final CRServo launcherServo;
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private boolean motorSpinning;
    private double activeLaunchSpeed;

    public launcher(HardwareMap hardwareMap){
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherServo = hardwareMap.get(CRServo.class, "launcherServo");
    }

    public void launch(double launchSpeed, double servoPower){
        if (!motorSpinning || activeLaunchSpeed != launchSpeed){
            launcherMotor.setPower(launchSpeed);
            activeLaunchSpeed = launchSpeed;
            spinUpTimer.reset();
            motorSpinning = true;
        }
        if (spinUpTimer.seconds() >= revSeconds) {
            launcherServo.setPower(servoPower);
        }
        else{
            launcherServo.setPower(0);
        }
    }

    public void stop() {
        launcherServo.setPower(0);
        launcherMotor.setPower(0);
        motorSpinning = false;
    }
}
