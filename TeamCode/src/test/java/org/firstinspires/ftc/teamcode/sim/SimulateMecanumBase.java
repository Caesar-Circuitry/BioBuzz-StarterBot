package org.firstinspires.ftc.teamcode.sim;

import com.pedropathing.algorithm.ForesightConfig;
import com.pedropathing.controllers.Controller;
import com.pedropathing.math.Matrix;
import com.pedropathing.math.Vector2D;
import org.codeblooded.ftcodesim.ascope.SeasonField;
import org.codeblooded.ftcodesim.ascope.SourceType;
import org.codeblooded.ftcodesim.hardware.SimHardwareMap;
import org.codeblooded.ftcodesim.hardware.devices.SimPinpoint;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedDrivetrain;
import org.codeblooded.ftcodesim.simulator.SimConfig;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimMecanumConfig;
import org.codeblooded.ftcodesim.input.DefaultKeybinds;
import org.codeblooded.ftcodesim.simulator.FTCodeSim;
import org.firstinspires.ftc.teamcode.sim.Util.LinearOpModeSupport;
import org.firstinspires.ftc.teamcode.sim.Util.MirroredMecanum;
import org.firstinspires.ftc.teamcode.sim.Util.SimIntakeConfig;
import org.firstinspires.ftc.teamcode.sim.Util.SimLauncherConfig;
import org.firstinspires.ftc.teamcode.sim.Util.SimulatedIntake;
import org.firstinspires.ftc.teamcode.sim.Util.SimulatedLauncher;
import org.firstinspires.ftc.teamcode.sim.Util.StarterBotGeometry;
import org.firstinspires.ftc.teamcode.sim.biobuzz.BioBuzzConfig;
import org.firstinspires.ftc.teamcode.sim.biobuzz.BioBuzzFieldSim;
import org.junit.Test;
import java.io.IOException;

public class SimulateMecanumBase {
    @Test
    public void test() throws IOException, InterruptedException {
        SimMecanumConfig config = new SimMecanumConfig();
        config.frontLeftMotorName = "FLDrive";
        config.frontRightMotorName = "FRDrive";
        config.backLeftMotorName = "RLDrive";
        config.backRightMotorName = "RRDrive";
        // Wheel layout and robot box, measured from AScope_Assets/Robot_StarterBot/model.glb:
        // 104 mm mecanums (radius 2.047), wheelbase 10.39, track width 16.30, frame 17.8 x 17.8
        // sitting 0.87 in ahead of the wheel centre. Sets wheelRadius, wheelbase, trackWidth, robotGeometry.
        StarterBotGeometry.applyTo(config);
        config.staticVelocityRegion = 4;
        config.staticFriction = 55;

        // TODO Fix 1 minute start up time

        config.maxAcceleration = 200;
        config.turnNaturalDeceleration = 2.5;
        config.maxTurnAccel = 10;
        config.maxAngularVelocity = 5;
        config.field = SeasonField.BIOBUZZ;

        config.robotModel = SourceType.ROBOT_MECANUM_BASE;

        config.foresightConfig = new ForesightConfig(
                c -> {
                    Controller primaryTranslationalForward = Controller.proportional(0.226493318780903);
                    Controller secondaryTranslationalForward = Controller.proportional(0.08368322927660432);
                    Controller primaryTranslationalLateral = Controller.proportional(0.5015381114306278);
                    Controller secondaryTranslationalLateral = Controller.proportional(0.18530493082846347);

                    c.forwardTranslational.set(Controller.piecewise(secondaryTranslationalForward).put(2.5, primaryTranslationalForward));
                    c.strafeTranslational.set(Controller.piecewise(secondaryTranslationalLateral).put(2.5, primaryTranslationalLateral));

                    c.coast.set(Controller.proportionalFeedforward(0.012957603483588636));
                    c.brake.set(Controller.proportionalFeedforward(0.01101396296105034));

                    c.headingFeedback.set(Controller.proportional(2.447421655157805));
                    c.headingBrakeCoefficients.set(Vector2D.cartesian(0.04730634263294, 0.007540928288975995));

                    c.linearBrakeCoefficients.set(Matrix.diag(0.06447, 0.06447));
                    c.quadraticBrakeCoefficients.set(Matrix.diag(0.0024, 0.0024));

                    c.maxAchievableForwardVelocity.set(82.80318372444954);
                    c.maxAchievableStrafeVelocity.set(66.35440133280454);
                    c.naturalForwardDeceleration.set(42.84137414187512);
                    c.naturalStrafeDeceleration.set(62.12942961650345);
                }
        );

        // StarterBot's right-side drive motors are mounted mirrored (its drive code negates them),
        // so tell the sim which ones; MirroredMecanum also honours setDirection and counts encoder ticks.
        SimulatedDrivetrain drivetrain = new MirroredMecanum(config, "FRDrive", "RRDrive");

        SimHardwareMap simHardwareMap = new SimHardwareMap();
        simHardwareMap.register(drivetrain);
        simHardwareMap.register("pinpoint", new SimPinpoint(drivetrain));

        // ---- Intake: creates the "intake" motor and the "intakeServo1"/"intakeServo2" CR servos ----
        SimIntakeConfig intakeConfig = new SimIntakeConfig();
        intakeConfig.motorName = "intake";
        intakeConfig.servoNames = new String[]{"intakeServo1", "intakeServo2"};
        intakeConfig.direction = 1;              // +power on the motor pulls balls in
        intakeConfig.storageCapacity = 4;        // balls the robot can hold
        intakeConfig.zoneDepth = 3;              // pickup box in front of the bumper (inches)
        intakeConfig.zoneWidth = 15;
        SimulatedIntake intake = new SimulatedIntake(intakeConfig);
        simHardwareMap.register(intake);

        // ---- Launcher: creates the "launcher" motor and the "launcherServo" feeder ----
        // 6000 RPM Yellow Jacket on a 96 mm Hogback wheel, 36 degree hood firing out the back.
        SimLauncherConfig launcherConfig = new SimLauncherConfig();
        launcherConfig.motorName = "launcher";
        launcherConfig.feederServoName = "launcherServo";
        launcherConfig.motorMaxRpm = 6000;
        launcherConfig.wheelDiameter = 3.78;     // inches
        launcherConfig.efficiency = 0.45;        // exit speed / wheel rim speed (TUNE)
        launcherConfig.launchAngleDeg = 36;
        launcherConfig.launchYawDeg = 180;       // 180 = out the back
        launcherConfig.exitX = -3.78;            // where the ball leaves, from the wheel centre
        launcherConfig.exitHeight = 10.5;
        SimulatedLauncher launcher = new SimulatedLauncher(launcherConfig);
        simHardwareMap.register(launcher);

        // ---- BioBuzz field: game pieces, hives, flowers ----
        // Register AFTER the drivetrain, intake and launcher. Field numbers are in BioBuzzConfig.
        BioBuzzConfig bioBuzz = new BioBuzzConfig();
        StarterBotGeometry.applyTo(bioBuzz);     // same robot box the drivetrain uses
        simHardwareMap.register(new BioBuzzFieldSim(drivetrain, intake, launcher, bioBuzz));

        // Lets LinearOpModes (like Teleop) run: FTCodeSim on its own only calls init()/loop(),
        // which are empty in a LinearOpMode. Attached to the sim below.
        LinearOpModeSupport linearOpModes = new LinearOpModeSupport();
        simHardwareMap.register(linearOpModes);

        SimConfig simConfig = new SimConfig();
        simConfig.gamepad1Keybinds = new DefaultKeybinds();
        simConfig.gamepad2Keybinds = new DefaultKeybinds();
        simConfig.simHardwareMap = simHardwareMap;
        simConfig.loopTimeMs = 10;
        simConfig.field = SeasonField.BIOBUZZ;
        simConfig.autoConfigureAscope = true;

        FTCodeSim sim = new FTCodeSim(simConfig);
        linearOpModes.attach(sim);
        sim.run();
    }
}