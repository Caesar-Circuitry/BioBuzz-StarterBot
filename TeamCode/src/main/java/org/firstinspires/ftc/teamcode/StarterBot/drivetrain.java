package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class drivetrain {
    public final DcMotorEx FLDriveMotor;
    public final DcMotorEx FRDriveMotor;
    public final DcMotorEx RLDriveMotor;
    public final DcMotorEx RRDriveMotor;
    public drivetrain(HardwareMap hardwareMap){
        FLDriveMotor = hardwareMap.get(DcMotorEx.class, "FLDrive");
        FRDriveMotor = hardwareMap.get(DcMotorEx.class, "FRDrive");
        RLDriveMotor = hardwareMap.get(DcMotorEx.class, "RLDrive");
        RRDriveMotor = hardwareMap.get(DcMotorEx.class, "RRDrive");
    }

    public void drive(double x, double y, double rx) {
        double frontLeft = y + x + rx;
        double frontRight = y - x - rx;
        double rearLeft = y - x + rx;
        double rearRight = y + x - rx;

        double maxPower = Math.max(
                1.0,
                Math.max(
                        Math.max(Math.abs(frontLeft), Math.abs(frontRight)),
                        Math.max(Math.abs(rearLeft), Math.abs(rearRight))));

        FLDriveMotor.setPower(frontLeft / maxPower);
        FRDriveMotor.setPower(frontRight / maxPower);
        RLDriveMotor.setPower(rearLeft / maxPower);
        RRDriveMotor.setPower(rearRight / maxPower);
    }
    public void stop(){
        FLDriveMotor.setPower(0);
        FRDriveMotor.setPower(0);
        RLDriveMotor.setPower(0);
        RRDriveMotor.setPower(0);
    }
}
