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
        FLDriveMotor.setPower(rx+(y+x));
        FRDriveMotor.setPower(rx-(y-x));
        RLDriveMotor.setPower(rx+(y-x));
        RRDriveMotor.setPower(rx-(y+x));
    }
    public void stop(){
        FLDriveMotor.setPower(0);
        FRDriveMotor.setPower(0);
        RLDriveMotor.setPower(0);
        RRDriveMotor.setPower(0);
    }
}
