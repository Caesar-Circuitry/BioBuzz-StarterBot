package org.firstinspires.ftc.teamcode.sim.Util;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ServoController;
import org.codeblooded.ftcodesim.hardware.devices.SimHardwareDevice;
import org.psilynx.psikit.core.Logger;

/**
 * Simulated continuous-rotation servo for FTCodeSim.
 *
 * Your robot code calls setPower()/setDirection() exactly like on the real robot.
 * Every physics tick FTCodeSim calls update(), which ramps the servo's actual
 * speed toward the commanded power. Read the result with getActualSpeed(),
 * getRpm() or getRotations(). Everything is also logged through PsiKit under
 * "Sim/CRServo/<name>/..." so it shows up in AdvantageScope next to the robot's logs.
 *
 * Register it in the sim test with the SAME name used in hardwareMap.get(...):
 *     simHardwareMap.register("intakeServo1", new SimCRServo("intakeServo1"));
 */
public class SimCRServo implements CRServo, SimHardwareDevice {
    private final String name;

    /** Free speed of the servo in RPM (goBILDA Speed ~150, Torque ~50, Axon Max+ ~110 no-load). */
    private final double maxRpm;

    /** Seconds to reach ~63% of a new speed. 0 = instant. */
    private final double spinUpTimeConstant;

    private double power = 0;                 // what the code asked for (-1..1)
    private Direction direction = Direction.FORWARD;

    private double actualSpeed = 0;           // simulated output (-1..1, after direction)
    private double rotations = 0;             // total output-shaft rotations since reset

    public SimCRServo(String name) {
        this(name, 150, 0.05);
    }

    public SimCRServo(String name, double maxRpm, double spinUpTimeConstant) {
        this.name = name;
        this.maxRpm = maxRpm;
        this.spinUpTimeConstant = spinUpTimeConstant;
    }

    // ---------- FTCodeSim hooks ----------

    @Override
    public void update(double deltaTime) {
        double target = direction == Direction.REVERSE ? -power : power;

        if (spinUpTimeConstant <= 0) {
            actualSpeed = target;
        } else {
            double alpha = Math.min(1.0, deltaTime / spinUpTimeConstant);
            actualSpeed += (target - actualSpeed) * alpha;
        }

        rotations += actualSpeed * (maxRpm / 60.0) * deltaTime;

        String key = "Sim/CRServo/" + name + "/";
        Logger.recordOutput(key + "CommandedPower", power);
        Logger.recordOutput(key + "Direction", direction.name());
        Logger.recordOutput(key + "ActualSpeed", actualSpeed);
        Logger.recordOutput(key + "Rpm", actualSpeed * maxRpm);
        Logger.recordOutput(key + "Rotations", rotations);
    }

    @Override
    public void reset() {
        power = 0;
        actualSpeed = 0;
        rotations = 0;
        direction = Direction.FORWARD;
    }

    // ---------- Extra getters for tests / custom sim mechanisms ----------

    /** Current simulated speed, -1..1, with direction applied. */
    public double getActualSpeed() { return actualSpeed; }

    public double getRpm() { return actualSpeed * maxRpm; }

    public double getRotations() { return rotations; }

    // ---------- CRServo / DcMotorSimple ----------

    @Override
    public void setPower(double power) {
        this.power = Math.max(-1.0, Math.min(1.0, power));
    }

    @Override
    public double getPower() {
        return power;
    }

    @Override
    public void setDirection(Direction direction) {
        this.direction = direction;
    }

    @Override
    public Direction getDirection() {
        return direction;
    }

    @Override
    public ServoController getController() {
        return null;
    }

    @Override
    public int getPortNumber() {
        return 0;
    }

    // ---------- HardwareDevice ----------

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Simulated CR Servo";
    }

    @Override
    public String getConnectionInfo() {
        return "sim:" + name;
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        reset();
    }

    @Override
    public void close() {
    }
}
