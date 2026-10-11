package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.constants.ControlConstants;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.robot.logging.PedroLogger;

public class Drivetrain {
    private enum Mode {
        PATH,
        MANUAL
    }
    private final Follower follower;
    private Mode mode = Mode.PATH;
    private double forward,strafe,turn;
    private boolean fieldCentric = true;

    public Drivetrain(Robot r, Pose startPose) {
        follower = Constants.create(r.hMap).withLogger(PedroLogger::logFollower);
        follower.setPose(startPose);
        follower.update();
    }
    public Follower follower() {return follower;}

    public void drive(double forward, double strafe, double turn) {
        this.forward = forward;
        this.strafe = strafe;
        this.turn = turn;
        mode = Mode.MANUAL;
    }

    public void setFieldCentric(boolean on) {fieldCentric = on;}
    public void usePathMode() {mode = Mode.PATH;}

    public Command periodic() {
        return infinite(() -> {
            if (mode == Mode.MANUAL) {
                if (fieldCentric) {
                    DrivePowers p = ManualDrive.fieldCentric(forward, strafe, turn,
                            follower.pose().heading() + ControlConstants.FIELD_HEADING_OFFSET_RAD);
                    ManualDrive.driveOrHold(follower, p);
                } else {
                    ManualDrive.driveOrHold(follower, forward,strafe,turn);
                }
            }
            follower.update();
            PedroLogger.logPose(follower.pose());
        });
    }
}
