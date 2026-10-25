package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.psilynx.psikit.core.Logger;

public class drivetrain {
    public final DcMotorEx FLDriveMotor;
    public final DcMotorEx FRDriveMotor;
    public final DcMotorEx RLDriveMotor;
    public final DcMotorEx RRDriveMotor;

    // The last request, kept so log() can show requested next to measured.
    private double requestedStrafe, requestedTurn, requestedForward;

    public drivetrain(HardwareMap hardwareMap){
        FLDriveMotor = hardwareMap.get(DcMotorEx.class, constants.FL_DRIVE);
        FRDriveMotor = hardwareMap.get(DcMotorEx.class, constants.FR_DRIVE);
        RLDriveMotor = hardwareMap.get(DcMotorEx.class, constants.RL_DRIVE);
        RRDriveMotor = hardwareMap.get(DcMotorEx.class, constants.RR_DRIVE);
    }

    public void drive(double x, double rx, double y){
        requestedStrafe = x;
        requestedTurn = rx;
        requestedForward = y;
        FLDriveMotor.setPower(rx+(y+x));
        FRDriveMotor.setPower(rx-(y-x));
        RLDriveMotor.setPower(rx+(y-x));
        RRDriveMotor.setPower(rx-(y+x));
    }
    public void stop(){
        drive(0, 0, 0);
    }

    /** Drive/... : the request, then what each motor was told and actually did. */
    public void log() {
        Logger.recordOutput("Drive/RequestedForward", requestedForward);
        Logger.recordOutput("Drive/RequestedStrafe", requestedStrafe);
        Logger.recordOutput("Drive/RequestedTurn", requestedTurn);
        logMotor("FL", FLDriveMotor);
        logMotor("FR", FRDriveMotor);
        logMotor("RL", RLDriveMotor);
        logMotor("RR", RRDriveMotor);
    }

    private static void logMotor(String name, DcMotorEx motor) {
        String key = "Drive/Motors/" + name + "/";
        Logger.recordOutput(key + "Power", motor.getPower());
        Logger.recordOutput(key + "VelocityTicksPerSec", motor.getVelocity());
        Logger.recordOutput(key + "PositionTicks", motor.getCurrentPosition());
        // A drive motor pulling much more current than the others usually means a binding wheel.
        Logger.recordOutput(key + "CurrentMilliamps", motor.getCurrent(CurrentUnit.MILLIAMPS));
    }
}
