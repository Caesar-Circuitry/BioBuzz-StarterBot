package org.firstinspires.ftc.teamcode.sim.Util;

import org.codeblooded.ftcodesim.hardware.SimHardwareMap;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.hardware.devices.SimMotorConfig;
import org.codeblooded.ftcodesim.hardware.devices.SimVoltageSensor;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimMecanumConfig;
import org.codeblooded.ftcodesim.hardware.drivetrain.SimulatedMecanum;

import java.util.Arrays;
import java.util.HashSet;
import java.util.Set;

/**
 * FTCodeSim's mecanum drivetrain, but with motors that behave like real ones:
 *
 *   - Mirrored mounting: on a real drivetrain the right-side motors face the
 *     other way, so +power spins them backwards. StarterBot's drive code allows
 *     for this by negating the right-side powers. Plain FTCodeSim treats +power
 *     as "forward" on every motor (and ignores setDirection), so that code made
 *     the robot spin instead of drive. List the mirrored motors here.
 *   - setDirection(REVERSE) is honoured.
 *   - getCurrentPosition() counts encoder ticks; getVelocity() returns ticks/s.
 *
 * <pre>
 *     SimulatedDrivetrain drivetrain = new MirroredMecanum(config, "FRDrive", "RRDrive");
 * </pre>
 */
public class MirroredMecanum extends SimulatedMecanum {
    private final Set<String> mirrored;
    /** Encoder ticks per wheel revolution (goBILDA 5203 312 RPM = 537.7). */
    public double ticksPerRev = 537.7;

    public MirroredMecanum(SimMecanumConfig config, String... mirroredMotorNames) {
        super(config);
        this.mirrored = new HashSet<>(Arrays.asList(mirroredMotorNames));
    }

    /** Same as FTCodeSim's version, but registers DirectionalSimMotors. */
    @Override
    public void registerDevices(SimHardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.voltageSensor = (SimVoltageSensor) hardwareMap.voltageSensor.iterator().next();
        for (int i = 0; i < motorNames.length; i++) {
            DirectionalSimMotor m = new DirectionalSimMotor(SimMotorConfig.empty(motorNames[i]),
                    mirrored.contains(motorNames[i]), ticksPerRev);
            motors[i] = hardwareMap.register(motorNames[i], m);
        }
    }

    /**
     * A SimMotor with a mounting direction, a code direction and an encoder.
     * The physics reads getPower() and writes `velocity` in "robot forward"
     * terms; the robot code sees the motor's own frame through setPower(),
     * getVelocity() and getCurrentPosition().
     */
    public static class DirectionalSimMotor extends SimMotor {
        private final boolean mountedMirrored;
        private final double ticksPerRev;
        private double commandedPower = 0;
        private double position = 0;
        private RunMode mode = RunMode.RUN_WITHOUT_ENCODER;

        public DirectionalSimMotor(SimMotorConfig config, boolean mountedMirrored, double ticksPerRev) {
            super(config);
            this.mountedMirrored = mountedMirrored;
            this.ticksPerRev = ticksPerRev;
        }

        /** +1 if +power drives the robot forward, -1 if it drives it backward. */
        private double sign() {
            double s = mountedMirrored ? -1 : 1;
            return direction == Direction.REVERSE ? -s : s;
        }

        @Override
        public void setDirection(Direction direction) {
            this.direction = direction;
        }

        @Override
        public void setPower(double power) {
            commandedPower = Math.max(-1, Math.min(1, power));
            super.setPower(commandedPower);
        }

        /** What the physics sees: power in "drive the robot forward" terms. */
        @Override
        public double getPower() {
            return commandedPower * sign();
        }

        @Override
        public void update(double deltaTime) {
            super.update(deltaTime);
            // velocity is the wheel speed in rad/s, robot-forward positive
            position += velocity * sign() * deltaTime * ticksPerRev / (2 * Math.PI);
        }

        @Override
        public void reset() {
            super.reset();
            commandedPower = 0;
            position = 0;
            direction = Direction.FORWARD;
        }

        @Override
        public int getCurrentPosition() {
            return (int) Math.round(position);
        }

        /** Encoder ticks per second, like a real DcMotorEx. */
        @Override
        public double getVelocity() {
            return velocity * sign() * ticksPerRev / (2 * Math.PI);
        }

        @Override
        public void setMode(RunMode mode) {
            if (mode == RunMode.STOP_AND_RESET_ENCODER) position = 0;
            this.mode = mode;
        }

        @Override
        public RunMode getMode() {
            return mode;
        }
    }
}
