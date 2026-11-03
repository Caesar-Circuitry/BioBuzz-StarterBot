package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class launcher{

    //commented things are for velocity
    // public static final double LAUNCH_VELOCITY = 1400;   // ticks/sec, tune this
    // private static final double VELOCITY_TOLERANCE = 50;
    private static final double revSeconds = 1.5;
    public static final double LAUNCH_POWER = 0.8;   // tune this
    public final DcMotorEx launcherMotor;
    public final CRServo launcherServo;
    private final ElapsedTime spinUpTimer = new ElapsedTime();
    private boolean motorSpinning;
    private double activeLaunchSpeed;

    public launcher(HardwareMap hardwareMap){
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherServo = hardwareMap.get(CRServo.class, "launcherServo");

        // launcherMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        // launcherMotor.setVelocityPIDFCoefficients(p, i, d, f);
    }

    public void rev(double launchSpeed){
        if (!motorSpinning || activeLaunchSpeed != launchSpeed){
            activeLaunchSpeed = launchSpeed;
            spinUpTimer.reset();
            motorSpinning = true;
        }
        launcherMotor.setPower(launchSpeed);
        // launcherMotor.setVelocity(launchSpeed);
    }
    public boolean isReady(){
        return motorSpinning && spinUpTimer.seconds() >= revSeconds;

        // return motorSpinning
        //         && Math.abs(launcherMotor.getVelocity() - activeLaunchSpeed) < VELOCITY_TOLERANCE;
    }
    public void feed(boolean buttonPressed, double servoPower){
        if (buttonPressed && isReady()) {
            launcherServo.setPower(servoPower);
        } else {
            launcherServo.setPower(0);
        }
    }

    public void stopFlywheel(){
        launcherMotor.setPower(0);
        // launcherMotor.setVelocity(0);
        motorSpinning = false;
    }
    public void stop() {
        launcherServo.setPower(0);
        stopFlywheel();
    }
}
