package org.firstinspires.ftc.teamcode.robot.control;

public class PidController {
    private final double kP, kI, kD, kF;

    private double integralLimit = Double.POSITIVE_INFINITY; // max size of the I term, in output units
    private double outMin = 0.0;
    private double outMax = 1.0;

    private double integral;
    private double lastMeasured;
    private boolean hasLast = false;

    private double pTerm, iTerm, dTerm, fTerm, output;

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

    public double update(double target, double measured, double dtSec) {
        double error = target - measured;

        // Feedforward: the guess, before any error exists
        fTerm = kF * target;

        // Proportional: push harder the further away we are
        pTerm = kP * error;

        // Integral: remove the leftover error over time, with a limit (anti-windup)
        integral += error * dtSec;
        if (kI != 0) {
            double maxIntegral = integralLimit / Math.abs(kI);
            integral = clamp(integral, -maxIntegral, maxIntegral);
        }
        iTerm = kI * integral;

        // Derivative on the measurement, so changing the target does not cause a spike
        dTerm = 0;
        if (hasLast && dtSec > 0) {
            dTerm = -kD * (measured - lastMeasured) / dtSec;
        }
        lastMeasured = measured;
        hasLast = true;

        output = clamp(fTerm + pTerm + iTerm + dTerm, outMin, outMax);
        return output;
    }

    public void reset() {
        integral = 0;
        hasLast = false;
        pTerm = iTerm = dTerm = fTerm = output = 0;
    }

    // For logging, so each term can be watched on its own
    public double pTerm()  { return pTerm; }
    public double iTerm()  { return iTerm; }
    public double dTerm()  { return dTerm; }
    public double fTerm()  { return fTerm; }
    public double output() { return output; }

    private static double clamp(double v, double lo, double hi) {
        return Math.max(lo, Math.min(hi, v));
    }
}