package org.firstinspires.ftc.teamcode.robot.control;

/**
 * A plain PID controller with feedforward. It turns "speed we want" and
 * "speed we have" into a motor power.
 *
 * Layer: robot.control, used by subsystems. This class is pure math on
 * purpose: it has no FTC SDK imports, so it can be read, reasoned about and
 * tested on a laptop without a robot.
 */
public class PidController {

    // ---- Fields ----

    private final double kP, kI, kD, kF;

    private double integralLimit = Double.POSITIVE_INFINITY; // max size of the I term, in output units
    private double outMin = 0.0;
    private double outMax = 1.0;

    private double integral;
    private double lastMeasured;
    private boolean hasLast = false;

    // Kept as fields only so the caller can log each term on its own.
    private double pTerm, iTerm, dTerm, fTerm, output;

    // ---- Constructor and setup ----

    public PidController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    public PidController withIntegralLimit(double limit) {
        this.integralLimit = limit;
        return this;
    }

    public PidController withOutputRange(double min, double max) {
        this.outMin = min;
        this.outMax = max;
        return this;
    }

    // ---- Update (call once per loop) ----

    public double update(double target, double measured, double dtSec) {
        double error = target - measured;

        // Feedforward: the guess, before any error exists
        fTerm = kF * target;

        // Proportional: push harder the further away we are
        pTerm = kP * error;

        // Integral: remove the leftover error over time, with a limit (anti-windup).
        // Without the limit, the sum keeps growing during spin-up (when the error
        // is big and there is nothing more the motor can do), and then it overshoots.
        // The limit is on the I TERM (in output units), so we divide by kI to get
        // the matching limit on the sum.
        integral += error * dtSec;
        if (kI != 0) {
            double maxIntegral = integralLimit / Math.abs(kI);
            integral = clamp(integral, -maxIntegral, maxIntegral);
        }
        iTerm = kI * integral;

        // Derivative on the MEASUREMENT, not on the error. When the target jumps
        // (launcher switched on: 0 -> full speed) the error jumps in one loop, and
        // "change in error / dt" would be a huge one-loop kick. The measured speed
        // cannot jump, so its slope stays smooth. The minus sign is because a rising
        // measurement means a falling error.
        dTerm = 0;
        // Skipped on the first loop after reset(): there is no previous
        // measurement yet, so there is no slope to compute.
        if (hasLast && dtSec > 0) {
            dTerm = -kD * (measured - lastMeasured) / dtSec;
        }
        lastMeasured = measured;
        hasLast = true;

        output = clamp(fTerm + pTerm + iTerm + dTerm, outMin, outMax);
        return output;
    }

    // Called while the mechanism is off, so old integral from the last run
    // does not push the motor the moment it is switched on again.
    public void reset() {
        integral = 0;
        hasLast = false;
        pTerm = iTerm = dTerm = fTerm = output = 0;
    }

    // ---- Getters ----

    // For logging, so each term can be watched on its own
    public double pTerm()  { return pTerm; }
    public double iTerm()  { return iTerm; }
    public double dTerm()  { return dTerm; }
    public double fTerm()  { return fTerm; }
    public double output() { return output; }

    // ---- Helpers ----

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}