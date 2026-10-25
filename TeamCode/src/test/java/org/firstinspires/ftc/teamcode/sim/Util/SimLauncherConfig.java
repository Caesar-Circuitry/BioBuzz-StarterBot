package org.firstinspires.ftc.teamcode.sim.Util;

/**
 * Settings for {@link SimulatedLauncher}, the same way SimMecanumConfig sets up the drivetrain.
 * Defaults are the StarterBot: goBILDA 5203-2402-0001 Yellow Jacket (1:1, 6000 RPM, 28 PPR)
 * driving a goBILDA Hogback Traction Wheel (96 mm, 82 g) directly, under a hood that rises
 * 36 degrees toward the back of the robot, fed by a CR-servo "windmill".
 * Distances in inches, robot frame: x forward from the drivetrain centre, y left, z up.
 */
public class SimLauncherConfig {
    // ---- Hardware (names must match hardwareMap.get(...) in the robot code) ----
    public String motorName = "launcher";
    public String feederServoName = "launcherServo";
    public double feederServoMaxRpm = 150;
    public double feederServoSpinUpTime = 0.05;
    /** Balls fed per full turn of the feeder (windmill: one per quarter turn). */
    public double feedsPerFeederTurn = 4;

    // ---- Flywheel ----
    public double motorMaxRpm = 6000;           // MOTOR: free speed at 12 V
    public double motorTicksPerRev = 28;        // MOTOR: what launcherMotor.getVelocity() counts
    public double gearRatio = 1;                // wheel turns per motor turn
    public double wheelDiameter = 3.78;         // HOGBACK: 96 mm
    /**
     * Seconds to reach ~63% of a new speed: wheel+hub inertia (~1.3e-4 kg m^2 for an 82 g wheel)
     * x free speed / stall torque (0.144 N m) = ~0.57 s.
     */
    public double spinUpTime = 0.57;
    /**
     * TUNE: ball exit speed as a fraction of the wheel's rim speed. A ball rolling between a
     * moving wheel and a fixed hood leaves at ~1/2 rim speed in theory; compression and slip
     * make it ~0.40-0.48. 0.45 -> ~535 in/s at full speed.
     */
    public double efficiency = 0.45;
    /** TUNE: fraction of wheel speed lost per shot (wheel stores ~26 J; a ball takes ~2 J + slip). */
    public double shotSpeedLoss = 0.08;

    // ---- Where and how the ball leaves ----
    /** STARTERBOT: the hood is a straight ramp rising 36 degrees. */
    public double launchAngleDeg = 36;
    /** STARTERBOT: balls leave over the BACK of the robot (0 = out the front, 180 = out the back). */
    public double launchYawDeg = 180;
    /** STARTERBOT: ball centre where it leaves the hood. */
    public double exitX = -3.78;
    public double exitY = 0;
    public double exitHeight = 10.5;
}
