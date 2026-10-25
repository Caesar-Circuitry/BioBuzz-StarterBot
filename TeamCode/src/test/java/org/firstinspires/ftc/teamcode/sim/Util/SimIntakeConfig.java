package org.firstinspires.ftc.teamcode.sim.Util;

/**
 * Settings for {@link SimulatedIntake}, the same way SimMecanumConfig sets up the drivetrain.
 * Defaults are the StarterBot (STARTERBOT = measured from AScope_Assets/Robot_StarterBot).
 * Distances in inches, robot frame: x forward from the drivetrain centre, y left.
 */
public class SimIntakeConfig {
    // ---- Hardware (names must match hardwareMap.get(...) in the robot code) ----
    public String motorName = "intake";
    public String[] servoNames = {"intakeServo1", "intakeServo2"};
    /** Intake motor free speed in encoder ticks/s at full power (goBILDA 5203 117 RPM: 1425.1 ticks/rev). */
    public double motorTicksPerSecAtFullPower = 117.0 / 60 * 1425.1;
    /** CR servo free speed and spin-up (goBILDA Speed servo ~150 RPM). */
    public double servoMaxRpm = 150;
    public double servoSpinUpTime = 0.05;

    // ---- Behaviour ----
    /** +1 if positive motor power pulls balls in (StarterBot INTAKE sets +1.0). */
    public int direction = 1;
    /** Below this |power| the intake does nothing. */
    public double minPower = 0.2;
    /** How many balls the robot can hold. */
    public int storageCapacity = 4;
    /** Seconds between pickups (one ball at a time). */
    public double cooldown = 0.12;
    /** Speed (in/s) a ball leaves the front when the intake runs backwards. */
    public double outtakeSpeed = 40;

    // ---- Pickup zone: a box in front of the front bumper ----
    /** STARTERBOT: rollers reach this far past the bumper. */
    public double zoneDepth = 3;
    /** STARTERBOT: 72 mm grip wheels at +-6.1 in plus their radius. */
    public double zoneWidth = 15;
    /** Balls whose bottom is higher than this are not grabbed. */
    public double maxBallHeight = 6;
}
