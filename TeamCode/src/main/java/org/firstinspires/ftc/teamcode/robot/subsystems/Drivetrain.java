package org.firstinspires.ftc.teamcode.robot.subsystems;

import static com.pedropathing.ivy.commands.Commands.infinite;

import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.math.Pose;

import org.firstinspires.ftc.teamcode.pedro.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;

public class Drivetrain {
    private final Follower follower;
    public Drivetrain(Robot r, Pose startPose) {
        follower = Constants.create(r.hMap);
        follower.setPose(startPose);
        follower.update();
    }

    public Command periodic() {
        return infinite(() -> {
            follower.update();
        });
    }
}
