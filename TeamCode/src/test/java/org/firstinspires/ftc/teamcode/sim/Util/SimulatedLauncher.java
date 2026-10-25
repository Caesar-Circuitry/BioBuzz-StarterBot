package org.firstinspires.ftc.teamcode.sim.Util;

import org.codeblooded.ftcodesim.hardware.SimHardwareMap;
import org.codeblooded.ftcodesim.hardware.devices.SimHardwareMechanism;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.hardware.devices.SimMotorConfig;
import org.psilynx.psikit.core.Logger;

/**
 * The launcher (flywheel + feeder) as a sim mechanism, set up like the drivetrain:
 * <pre>
 *     SimLauncherConfig launcherConfig = new SimLauncherConfig();
 *     SimulatedLauncher launcher = new SimulatedLauncher(launcherConfig);
 *     simHardwareMap.register(launcher);   // creates the launcher motor and feeder servo
 * </pre>
 * Every physics tick it spins the flywheel toward the motor's power, writes the
 * speed back so launcherMotor.getVelocity() works (ticks/s), and counts balls the
 * feeder pushes in. The BioBuzz field takes those shots and launches the balls.
 */
public class SimulatedLauncher implements SimHardwareMechanism {
    public final SimLauncherConfig config;
    public final LauncherPhysics physics;
    private SimMotor motor;
    private SimCRServo feeder;
    private int pendingShots = 0;

    public SimulatedLauncher(SimLauncherConfig config) {
        this.config = config;
        this.physics = new LauncherPhysics(config);
    }

    @Override
    public void registerDevices(SimHardwareMap hardwareMap) {
        motor = hardwareMap.motor(SimMotorConfig.empty(config.motorName));
        feeder = hardwareMap.register(config.feederServoName,
                new SimCRServo(config.feederServoName, config.feederServoMaxRpm, config.feederServoSpinUpTime));
    }

    @Override
    public void update(double deltaTime) {
        if (motor == null) return;
        pendingShots += physics.update(deltaTime, motor.getPower(), feeder.getRotations());
        motor.velocity = physics.ticksPerSecond();
        Logger.recordOutput("Sim/Launcher/FlywheelRpm", physics.rpm());
        Logger.recordOutput("Sim/Launcher/FlywheelTicksPerSec", physics.ticksPerSecond());
        Logger.recordOutput("Sim/Launcher/ExitSpeedInPerSec", physics.exitSpeed());
    }

    /** Balls fed since the last call. */
    public int takeShots() {
        int n = pendingShots;
        pendingShots = 0;
        return n;
    }

    public double exitSpeed() { return physics.exitSpeed(); }

    public void onShot() { physics.onShot(); }

    public double rpm() { return physics.rpm(); }

    @Override
    public void reset() {
        physics.reset();
        pendingShots = 0;
    }
}
