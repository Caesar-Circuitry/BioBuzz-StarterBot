package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
public class drivetrain {
    public final DcMotorEx FLDriveMotor;
    public final DcMotorEx FRDriveMotor;

    public drivetrain(HardwareMap hardwareMap){
        FLDriveMotor = hardwareMap.get(DcMotorEx.class, "FLDrive");
        FRDriveMotor = hardwareMap.get(DcMotorEx.class, "FRDrive");
    }
}
