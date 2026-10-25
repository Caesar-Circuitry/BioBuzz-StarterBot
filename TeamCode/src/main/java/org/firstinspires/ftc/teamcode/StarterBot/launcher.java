package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.psilynx.psikit.core.Logger;

public class launcher {
    public final DcMotorEx launcherMotor;
    public final CRServo launcherServo;

    public enum Mode { OFF, STANDARD, VARIABLE }
    private Mode mode = Mode.OFF;

    public launcher(HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, constants.LAUNCHER_MOTOR);
        launcherServo = hardwareMap.get(CRServo.class, constants.LAUNCHER_SERVO);
    }
    public void launchVARIABLE(double launchSpeed, double ServoPower){
        mode = Mode.VARIABLE;
        launcherMotor.setPower(launchSpeed);
        launcherServo.setPower(ServoPower);
    }
    public void launchSTANDARD(){
        mode = Mode.STANDARD;
        launcherMotor.setPower(constants.LAUNCH_STANDARD_FLYWHEEL_POWER);
        launcherServo.setPower(constants.LAUNCH_STANDARD_WINDMILL_POWER);//IDK what a 'standard' launch speed is
    }
    public void stop(){
        mode = Mode.OFF;
        launcherMotor.setPower(0);
        launcherServo.setPower(0);
    }

    public Mode getMode() {
        return mode;
    }

    /**
     * Shooter/... : powers we asked for next to the flywheel speed we got. Feeding
     * while MeasuredSpeed is still climbing is why shots come out short.
     */
    public void log() {
        Logger.recordOutput("Shooter/Mode", mode);
        Logger.recordOutput("Shooter/LauncherPower", launcherMotor.getPower());
        Logger.recordOutput("Shooter/WindmillPower", launcherServo.getPower());
        Logger.recordOutput("Shooter/MeasuredSpeedTicksPerSec", launcherMotor.getVelocity());
        Logger.recordOutput("Shooter/CurrentMilliamps", launcherMotor.getCurrent(CurrentUnit.MILLIAMPS));
    }
}
