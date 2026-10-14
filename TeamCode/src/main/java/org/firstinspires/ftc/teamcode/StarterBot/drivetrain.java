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

    public void drive(double x, double rx, double y){
        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        double frontLeftPower = (y + x + rx) / denominator;
        double backLeftPower = (y - x + rx) / denominator;
        double frontRightPower = (y - x - rx) / denominator;
        double backRightPower = (y + x - rx) / denominator;

        FLDriveMotor.setPower(frontLeftPower);
        FRDriveMotor.setPower(frontRightPower);
        RLDriveMotor.setPower(backLeftPower);
        RRDriveMotor.setPower(backRightPower);
    }
    public void stop(){
        FLDriveMotor.setPower(0);
        FRDriveMotor.setPower(0);
        RLDriveMotor.setPower(0);
        RRDriveMotor.setPower(0);
    }


}
