package org.firstinspires.ftc.teamcode.StarterBot;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp - Distance Speed", group = "Competition")
public class TeleopDistance extends StarterTeleopBase {
    @Override
    protected boolean useDistanceSpeed() { return true; }
}