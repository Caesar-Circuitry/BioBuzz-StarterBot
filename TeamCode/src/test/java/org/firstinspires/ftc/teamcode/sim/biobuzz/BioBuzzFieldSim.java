package org.firstinspires.ftc.teamcode.sim.biobuzz;

import org.codeblooded.ftcodesim.ascope.boundaries.MotionVector;
import org.codeblooded.ftcodesim.hardware.devices.SimHardwareMechanism;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedDrivetrain;
import org.firstinspires.ftc.teamcode.sim.Util.SimulatedIntake;
import org.firstinspires.ftc.teamcode.sim.Util.SimulatedLauncher;

/**
 * Plugs the BioBuzz field into FTCodeSim.
 *
 * FTCodeSim calls update() every physics tick (about every millisecond). Each
 * tick this reads the drivetrain's pose and velocity and the intake's power,
 * steps the field physics, sends a ball out for every shot the launcher's feeder
 * pushed in, and draws everything in AdvantageScope.
 *
 * Register it AFTER the drivetrain, intake and launcher:
 * <pre>
 *     simHardwareMap.register(drivetrain);
 *     simHardwareMap.register(intake);
 *     simHardwareMap.register(launcher);
 *     simHardwareMap.register(new BioBuzzFieldSim(drivetrain, intake, launcher, bioBuzzConfig));
 * </pre>
 */
public class BioBuzzFieldSim implements SimHardwareMechanism {
    public final BioBuzzField field;
    public final BioBuzzConfig config;
    public final BioBuzzVisualizer visualizer;
    private final SimulatedDrivetrain drivetrain;
    private final SimulatedIntake intake;
    private final SimulatedLauncher launcher;

    /** Print launches, pickups and tips to the Run window. */
    public boolean printEvents = true;

    public BioBuzzFieldSim(SimulatedDrivetrain drivetrain, SimulatedIntake intake,
                           SimulatedLauncher launcher, BioBuzzConfig config) {
        this.drivetrain = drivetrain;
        this.intake = intake;
        this.launcher = launcher;
        this.config = config;
        this.field = new BioBuzzField(config, intake.config, launcher.config);
        this.visualizer = new BioBuzzVisualizer();
        BioBuzzAssets.install();
        moveRobotToStart();
        printLayoutCheck();
    }

    /** Print whether the starting layout matches the manual (see BioBuzzField.validate()). */
    public void printLayoutCheck() {
        java.util.List<String> problems = field.validate();
        if (problems.isEmpty()) {
            System.out.println("[BioBuzz] Layout check passed: " + field.pollen.size() + " pollen, "
                    + field.redNectar.size() + " red nectar, " + field.blueNectar.size() + " blue nectar");
        } else {
            System.out.println("[BioBuzz] Layout check found " + problems.size() + " problem(s):");
            for (String p : problems) System.out.println("[BioBuzz]   - " + p);
        }
    }

    @Override
    public void update(double deltaTime) {
        RobotInput in = new RobotInput();
        in.x = drivetrain.position.x;
        in.y = drivetrain.position.y;
        in.heading = drivetrain.position.theta;
        in.vx = drivetrain.velocity.x;
        in.vy = drivetrain.velocity.y;
        in.omega = drivetrain.velocity.theta;
        in.intakePower = intake.power();

        field.step(deltaTime, in);

        // every ball the feeder pushed into the flywheel this tick
        for (int shots = launcher.takeShots(); shots > 0; shots--) {
            if (field.launch(launcher.exitSpeed())) launcher.onShot();
        }

        if (printEvents) {
            for (String e : field.drainEvents()) System.out.println("[BioBuzz] " + e);
        } else {
            field.drainEvents();
        }

        visualizer.update(field, deltaTime);
    }

    /** Called by FTCodeSim when an OpMode stops: put every piece and the robot back. */
    @Override
    public void reset() {
        field.reset();
        moveRobotToStart();
        printLayoutCheck();
        System.out.println("[BioBuzz] Field reset");
    }

    private void moveRobotToStart() {
        drivetrain.setPosition(new MotionVector(config.startX, config.startY, config.startHeading));
        drivetrain.velocity = new MotionVector(0, 0, 0);
    }
}
