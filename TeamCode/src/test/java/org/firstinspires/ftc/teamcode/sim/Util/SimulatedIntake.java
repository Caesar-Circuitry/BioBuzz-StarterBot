package org.firstinspires.ftc.teamcode.sim.Util;

import org.codeblooded.ftcodesim.hardware.SimHardwareMap;
import org.codeblooded.ftcodesim.hardware.devices.SimHardwareMechanism;
import org.codeblooded.ftcodesim.hardware.devices.SimMotor;
import org.codeblooded.ftcodesim.hardware.devices.SimMotorConfig;
import org.psilynx.psikit.core.Logger;

/**
 * The intake as a sim mechanism, set up like the drivetrain:
 * <pre>
 *     SimIntakeConfig intakeConfig = new SimIntakeConfig();
 *     SimulatedIntake intake = new SimulatedIntake(intakeConfig);
 *     simHardwareMap.register(intake);     // creates the intake motor and CR servos
 * </pre>
 * It owns the hardware the robot code reads and writes. Picking balls up is done by
 * the BioBuzz field (it knows where the balls are), using this intake's power and
 * its config's pickup zone.
 */
public class SimulatedIntake implements SimHardwareMechanism {
    public final SimIntakeConfig config;
    private SimMotor motor;
    private SimCRServo[] servos = new SimCRServo[0];

    public SimulatedIntake(SimIntakeConfig config) {
        this.config = config;
    }

    @Override
    public void registerDevices(SimHardwareMap hardwareMap) {
        motor = hardwareMap.motor(SimMotorConfig.empty(config.motorName));
        servos = new SimCRServo[config.servoNames.length];
        for (int i = 0; i < servos.length; i++) {
            servos[i] = hardwareMap.register(config.servoNames[i],
                    new SimCRServo(config.servoNames[i], config.servoMaxRpm, config.servoSpinUpTime));
        }
    }

    /** Intake effort, -1..1, positive = pulling balls in. */
    public double power() {
        return motor == null ? 0 : motor.getPower() * config.direction;
    }

    @Override
    public void update(double deltaTime) {
        if (motor == null) return;
        // so intakeMotor.getVelocity() reads something sensible in the robot code
        motor.velocity = motor.getPower() * config.motorTicksPerSecAtFullPower;
        Logger.recordOutput("Sim/Intake/Power", power());
        Logger.recordOutput("Sim/Intake/Running", Math.abs(power()) >= config.minPower);
    }
}
