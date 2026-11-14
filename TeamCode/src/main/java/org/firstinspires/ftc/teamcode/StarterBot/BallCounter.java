package org.firstinspires.ftc.teamcode.StarterBot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Configurable
public class BallCounter {
    public static double BALL_PRESENT_CM = 8;
    public static double BALL_GONE_CM = 12;
    public static int MAX_BALLS = 4;
    public static double MIN_GAP_SECONDS = 0.15;
    private static final double KALMAN_Q = 10;
    private static final double KALMAN_R = 4;
    private static final double MAX_RANGE_CM = 50;

    private final DistanceSensor sensor;
    private final KalmanFilter distanceFilter = new KalmanFilter(KALMAN_Q, KALMAN_R);
    private final ElapsedTime sinceLastCount = new ElapsedTime();

    private boolean ballPresent = false;
    private int count = 0;
    private double rawDistanceCm = 0;
    private double distanceCm = 0;

    public BallCounter(HardwareMap hardwareMap) {
        sensor = hardwareMap.get(DistanceSensor.class, "intakeSensor");
    }

    public void update() {
        rawDistanceCm = Math.min(sensor.getDistance(DistanceUnit.CM), MAX_RANGE_CM);
        distanceCm = distanceFilter.update(rawDistanceCm);

        if (!ballPresent && distanceCm < BALL_PRESENT_CM) {
            ballPresent = true;
            if (sinceLastCount.seconds() >= MIN_GAP_SECONDS) {
                count = Math.min(count + 1, MAX_BALLS);
                sinceLastCount.reset();
            }
        } else if (ballPresent && distanceCm > BALL_GONE_CM) {
            ballPresent = false;
        }
    }

    public void removeOne()            { count = Math.max(count - 1, 0); }

    public void reset()                { count = 0; }
    public void setCount(int n)        { count = Math.max(0, Math.min(n, MAX_BALLS)); }
    public int getCount()              { return count; }
    public boolean isFull()            { return count >= MAX_BALLS; }
    public boolean isBallPresent()     { return ballPresent; }
    public double getDistanceCm()      { return distanceCm; }
    public double getRawDistanceCm()   { return rawDistanceCm; }
}