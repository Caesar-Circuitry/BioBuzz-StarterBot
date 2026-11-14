package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp - Fixed Speed", group = "Competition")
public class TeleopFixed extends StarterTeleopBase {
    @Override
    protected boolean useDistanceSpeed() { return false; }
}