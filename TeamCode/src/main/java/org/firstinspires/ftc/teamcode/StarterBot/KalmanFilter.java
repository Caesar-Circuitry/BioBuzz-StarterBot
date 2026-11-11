package org.firstinspires.ftc.teamcode.StarterBot;

public class KalmanFilter {
    private double q;
    private double r;
    private double estimate = 0;
    private double errorCovariance = 1;
    private boolean initialized = false;

    public KalmanFilter(double q, double r) {
        this.q = q;
        this.r = r;
    }

    public void setNoise(double q, double r) {
        this.q = q;
        this.r = r;
    }

    public double update(double measurement) {
        if (!initialized) {
            estimate = measurement;
            errorCovariance = r;
            initialized = true;
            return estimate;
        }

        errorCovariance += q;

        double gain = errorCovariance / (errorCovariance + r);
        estimate += gain * (measurement - estimate);
        errorCovariance *= (1 - gain);

        return estimate;
    }

    public void reset() {
        initialized = false;
        errorCovariance = 1;
    }

    public double getEstimate() {
        return estimate;
    }
}
