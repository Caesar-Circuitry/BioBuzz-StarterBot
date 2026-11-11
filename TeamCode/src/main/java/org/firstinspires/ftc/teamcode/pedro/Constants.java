package org.firstinspires.ftc.teamcode.pedro;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import com.pedropathing.revhub.drivetrains.MecanumConfig;
import com.pedropathing.revhub.localizers.PinpointConfig;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {

    public static Follower create(HardwareMap h) {
        // return new Follower(Drivetrain, Localizer, Foresight);
        return null;
    }
    public static MecanumConfig drivetrainConfig = new MecanumConfig(c -> {
        c.frontLeftName.set("FLDrive");
        c.frontRightName.set("FRDrive");
        c.backLeftName.set("RLDrive");
        c.backRightName.set("RRDrive");
        c.frontLeftDirection.set(DcMotorSimple.Direction.REVERSE);
        c.frontRightDirection.set(DcMotorSimple.Direction.REVERSE);
        c.backLeftDirection.set(DcMotorSimple.Direction.FORWARD);
        c.backRightDirection.set(DcMotorSimple.Direction.REVERSE);
    });
    public static PinpointConfig localizerConfig = new PinpointConfig(c -> {
        c.name.set("pinpoint");
        c.podType.set(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        c.xPodOffset.set(-5.136838297205647);
        c.yPodOffset.set(2.6264584158349225);
        c.xPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.yPodDirection.set(GoBildaPinpointDriver.EncoderDirection.FORWARD);
        c.globalDistanceUnit.set(DistanceUnit.INCH);
        c.offsetUnits.set(DistanceUnit.INCH);
    });
    public static ForesightConfig foresightConfig = new ForesightConfig(
            c -> {
                Controller primaryTranslationalForward = Controller.proportional(0.29654203405317686);
                Controller secondaryTranslationalForward = Controller.proportional(0.10956435783356516);
                Controller primaryTranslationalLateral = Controller.proportional(0.41341324319212847);
                Controller secondaryTranslationalLateral = Controller.proportional(0.15274514675418513);

                c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                c.coast.set(Controller.proportionalFeedforward(0.011935627167558282));
                c.brake.set(Controller.proportionalFeedforward(0.01014528309242454));

                c.headingFeedback.set(Controller.proportional(5.71174229708817));
                c.headingBrakeCoefficients.set(Vector2D.cartesian(0.0686629152573269, 0.004039516230710332));

                c.linearBrakeCoefficients.set(Matrix.diag(0.11381928006913303, 0.07743507953044255));
                c.quadraticBrakeCoefficients.set(Matrix.diag(9.361505865952235E-4, 0.0010773275834462775));

                c.maxAchievableForwardVelocity.set(76.84352581564971);
                c.maxAchievableStrafeVelocity.set(62.39996730158854);
                c.naturalForwardDeceleration.set(30.11367680365669);
                c.naturalStrafeDeceleration.set(58.06420692252943);
            }
    );
}