package org.firstinspires.ftc.teamcode.sim.Util;

/**
 * The flywheel + feeder model on its own (plain Java, no FTCodeSim), so it can be
 * tested and reused. {@link SimulatedLauncher} drives it from the real devices.
 */
public class LauncherPhysics {
    public final SimLauncherConfig config;
    private double omega = 0;                    // motor speed, rad/s
    private double lastFeederRotations = Double.NaN;
    private double feedProgress = 0;

    public LauncherPhysics(SimLauncherConfig config) {
        this.config = config;
    }

    /** Advance the flywheel and feeder. Returns how many balls the feeder pushed in this step. */
    public int update(double dt, double motorPower, double feederRotations) {
        double maxOmega = config.motorMaxRpm * 2 * Math.PI / 60;
        double target = Math.max(-1, Math.min(1, motorPower)) * maxOmega;
        double alpha = config.spinUpTime <= 0 ? 1 : Math.min(1, dt / config.spinUpTime);
        omega += (target - omega) * alpha;

        if (Double.isNaN(lastFeederRotations)) {
            lastFeederRotations = feederRotations;
            return 0;
        }
        double delta = feederRotations - lastFeederRotations;
        lastFeederRotations = feederRotations;
        if (delta <= 0) return 0;                     // reversing the feeder doesn't launch
        feedProgress += delta * config.feedsPerFeederTurn;
        int shots = (int) Math.floor(feedProgress);
        feedProgress -= shots;
        return shots;
    }

    /** Ball exit speed (in/s) right now. */
    public double exitSpeed() {
        return Math.abs(omega) * config.gearRatio * config.wheelDiameter / 2 * config.efficiency;
    }

    /** A ball went through: it takes some of the wheel's speed. */
    public void onShot() {
        omega *= (1 - config.shotSpeedLoss);
    }

    public double rpm() { return omega * 60 / (2 * Math.PI); }

    /** Motor speed the way DcMotorEx.getVelocity() reports it: encoder ticks per second. */
    public double ticksPerSecond() { return omega / (2 * Math.PI) * config.motorTicksPerRev; }

    public void reset() {
        omega = 0;
        lastFeederRotations = Double.NaN;
        feedProgress = 0;
    }
}
