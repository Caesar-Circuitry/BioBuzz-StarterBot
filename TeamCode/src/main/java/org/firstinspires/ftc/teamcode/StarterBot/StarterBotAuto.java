/*
 * Copyright (c) 2026 Base 10 Assets, LLC
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.
 *
 * Neither the name of NAME nor the names of its contributors may be used to
 * endorse or promote products derived from this software without specific prior
 * written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR
 * TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.StarterBot;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.psilynx.psikit.core.Logger;

/**
 * Drives forward DRIVE_DISTANCE_MM and stops. Now a PsiKit OpMode (see
 * StarterBotOpMode): progress is logged under Auto/...
 */
@Disabled
@Autonomous(name = "StarterBotAuto", group = "StarterBot")
public class StarterBotAuto extends StarterBotOpMode {
    private static final double DRIVE_SPEED = 0.5;
    private static final double DRIVE_DISTANCE_MM = 600;
    private static final double WHEEL_DIAMETER_MM = 96;
    private static final double ENCODER_TICKS_PER_REV = 537.7;
    private static final double TICKS_PER_MM =
            ENCODER_TICKS_PER_REV / (WHEEL_DIAMETER_MM * Math.PI);
    private static final double HOLD_SECONDS = 0.2;

    private final ElapsedTime driveTimer = new ElapsedTime();
    private DcMotor leftDrive;
    private DcMotor rightDrive;
    private boolean complete;
    private int targetTicks;

    @Override protected String modeName() {
        return "StarterBotAuto";
    }

    @Override protected void onPsiKitConfigureLogging() {
        super.onPsiKitConfigureLogging();
        Logger.recordMetadata("Auto/DriveSpeed", String.valueOf(DRIVE_SPEED));
        Logger.recordMetadata("Auto/DriveDistanceMm", String.valueOf(DRIVE_DISTANCE_MM));
        Logger.recordMetadata("Auto/WheelDiameterMm", String.valueOf(WHEEL_DIAMETER_MM));
        Logger.recordMetadata("Auto/EncoderTicksPerRev", String.valueOf(ENCODER_TICKS_PER_REV));
        Logger.recordMetadata("Auto/HoldSeconds", String.valueOf(HOLD_SECONDS));
    }

    @Override
    protected void onPsiKitInit() {
        super.onPsiKitInit();
        leftDrive = hardwareMap.get(DcMotor.class, "left_drive");
        rightDrive = hardwareMap.get(DcMotor.class, "right_drive");

        leftDrive.setDirection(DcMotor.Direction.FORWARD);
        rightDrive.setDirection(DcMotor.Direction.REVERSE);
        leftDrive.setZeroPowerBehavior(BRAKE);
        rightDrive.setZeroPowerBehavior(BRAKE);

        leftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        telemetry.addData("Status", "Ready to drive forward %.0f mm", DRIVE_DISTANCE_MM);
    }

    @Override
    protected void onPsiKitStart() {
        driveTimer.reset();
    }

    @Override
    protected void onPsiKitLoop() {
        if (!complete && drive(DRIVE_SPEED, DRIVE_DISTANCE_MM, HOLD_SECONDS)) {
            stopDrive();
            complete = true;
        }

        telemetry.addData("Status", complete ? "Auto complete" : "Driving forward");
        telemetry.addData("Left position", leftDrive.getCurrentPosition());
        telemetry.addData("Right position", rightDrive.getCurrentPosition());
        telemetry.update();

        Logger.recordOutput("Auto/Complete", complete);
        Logger.recordOutput("Auto/TargetTicks", targetTicks);
        Logger.recordOutput("Auto/LeftPositionTicks", leftDrive.getCurrentPosition());
        Logger.recordOutput("Auto/RightPositionTicks", rightDrive.getCurrentPosition());
        Logger.recordOutput("Auto/LeftErrorTicks", targetTicks - leftDrive.getCurrentPosition());
        Logger.recordOutput("Auto/RightErrorTicks", targetTicks - rightDrive.getCurrentPosition());
        Logger.recordOutput("Auto/TimeAtTargetSec", driveTimer.seconds());
        logRobot();
    }

    @Override
    protected void onPsiKitStop() {
        if (leftDrive != null) stopDrive();
    }

    private boolean drive(double speed, double distanceMm, double holdSeconds) {
        targetTicks = (int) (distanceMm * TICKS_PER_MM);
        leftDrive.setTargetPosition(targetTicks);
        rightDrive.setTargetPosition(targetTicks);
        leftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        rightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftDrive.setPower(speed);
        rightDrive.setPower(speed);

        double toleranceTicks = 10 * TICKS_PER_MM;
        boolean leftAtTarget =
                Math.abs(targetTicks - leftDrive.getCurrentPosition()) <= toleranceTicks;
        boolean rightAtTarget =
                Math.abs(targetTicks - rightDrive.getCurrentPosition()) <= toleranceTicks;
        if (!leftAtTarget || !rightAtTarget) {
            driveTimer.reset();
        }

        return driveTimer.seconds() >= holdSeconds;
    }

    private void stopDrive() {
        leftDrive.setPower(0);
        rightDrive.setPower(0);
    }
}
