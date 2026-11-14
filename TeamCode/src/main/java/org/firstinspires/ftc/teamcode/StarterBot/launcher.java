package org.firstinspires.ftc.teamcode.StarterBot;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@Configurable   // makes the public static values editable in Panels
public class launcher {
    // ===== TUNING (editable live in Panels) =====
    public static double TARGET_VELOCITY = 1400;   // ticks/sec (6000rpm motor max is about 2800)
    public static double kP = 0.001;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kF = 0.00036;             // power per tick/sec at 12V (about 1/2800)
    public static double READY_TOLERANCE = 50;     // ticks/sec from target that counts as "ready"
    public static double READY_HOLD_SECONDS = 0.15; // must STAY in tolerance this long before "ready"
    public static double INTEGRAL_LIMIT = 500;     // stops the I term from building up forever
    public static double SHOT_DIP = 150;           // ticks/sec drop below target that means a ball went through

    // ===== Kalman filter (fixed, not in Panels) =====
    private static final double KALMAN_Q = 50;     // bigger = reacts faster, less smoothing
    private static final double KALMAN_R = 1500;   // bigger = smoother, but more lag

    private static final double NOMINAL_VOLTAGE = 12.0;
    private static final double VOLTAGE_READ_SECONDS = 0.25;   // voltage reads are slow, don't do every loop

    public final DcMotorEx launcherMotor;
    public final CRServo launcherServo;
    private final VoltageSensor voltageSensor;
    private final KalmanFilter velocityFilter = new KalmanFilter(KALMAN_Q, KALMAN_R);
    private final ElapsedTime loopTimer = new ElapsedTime();
    private final ElapsedTime voltageTimer = new ElapsedTime();

    // flywheel state
    private boolean motorSpinning = false;
    private double targetVelocity = 0;
    private double rawVelocity = 0;
    private double filteredVelocity = 0;
    private double batteryVoltage = NOMINAL_VOLTAGE;
    private double integral = 0;
    private double lastError = 0;
    private double motorPower = 0;

    // feed state
    private boolean feeding = false;           // is the feed toggled on?
    private boolean lastButtonState = false;   // for detecting a new press
    private boolean feedBlocked = false;       // true = pause feeding (robot got pushed off its spot)

    // shot detection
    private boolean shotInProgress = false;
    private boolean shotDetected = false;

    // ready = in tolerance continuously for READY_HOLD_SECONDS (stops flickering and early feeding)
    private final ElapsedTime inToleranceTimer = new ElapsedTime();
    private boolean inTolerance = false;

    public launcher(HardwareMap hardwareMap) {
        launcherMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        launcherServo = hardwareMap.get(CRServo.class, "launcherServo");
        launcherMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcherMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        voltageSensor = hardwareMap.voltageSensor.iterator().next();
        batteryVoltage = voltageSensor.getVoltage();
    }

    public void rev(double velocity) {
        if (!motorSpinning || velocity != targetVelocity) {
            integral = 0;
            lastError = velocity - filteredVelocity;
        }
        targetVelocity = velocity;
        motorSpinning = true;
    }

    public void update() {
        double dt = loopTimer.seconds();
        loopTimer.reset();
        if (dt <= 0 || dt > 0.5) dt = 0.02;
        rawVelocity = launcherMotor.getVelocity();
        filteredVelocity = velocityFilter.update(rawVelocity);

        boolean nowInTolerance = motorSpinning
                && Math.abs(targetVelocity - filteredVelocity) < READY_TOLERANCE;
        if (nowInTolerance && !inTolerance) {
            inToleranceTimer.reset();
        }
        inTolerance = nowInTolerance;

        boolean dipping = motorSpinning && feeding && (targetVelocity - rawVelocity) > SHOT_DIP;
        if (dipping && !shotInProgress) {
            shotDetected = true;
        }
        shotInProgress = dipping;

        if (voltageTimer.seconds() >= VOLTAGE_READ_SECONDS) {
            double v = voltageSensor.getVoltage();
            if (v > 1) batteryVoltage = v;
            voltageTimer.reset();
        }

        if (!motorSpinning) {
            motorPower = 0;
            launcherMotor.setPower(0);
            return;
        }

        double error = targetVelocity - filteredVelocity;
        integral = Range.clip(integral + error * dt, -INTEGRAL_LIMIT, INTEGRAL_LIMIT);
        double derivative = (error - lastError) / dt;
        lastError = error;

        double output = kF * targetVelocity
                + kP * error
                + kI * integral
                + kD * derivative;

        output *= NOMINAL_VOLTAGE / batteryVoltage;

        motorPower = Range.clip(output, 0, 1);
        launcherMotor.setPower(motorPower);
    }

    public boolean isReady() {
        return inTolerance && inToleranceTimer.seconds() >= READY_HOLD_SECONDS;
    }

    public void feed(boolean button, double servoPower) {
        if (button && !lastButtonState) {
            feeding = !feeding;
        }
        lastButtonState = button;

        if (feeding && isReady() && !feedBlocked) {
            launcherServo.setPower(servoPower);
        } else {
            launcherServo.setPower(0);
        }
    }

    public void setFeedBlocked(boolean blocked) {
        feedBlocked = blocked;
    }
    public void setFeeding(boolean on) {
        feeding = on;
    }
    public void stopFeeding() {
        feeding = false;
    }

    public void stopFlywheel() {
        motorSpinning = false;
        feeding = false;
        integral = 0;
        launcherMotor.setPower(0);
    }

    public void stop() {
        launcherServo.setPower(0);
        stopFlywheel();
    }

    public boolean consumeShot() {
        boolean shot = shotDetected;
        shotDetected = false;
        return shot;
    }

    public double getRawVelocity()      { return rawVelocity; }
    public double getFilteredVelocity() { return filteredVelocity; }
    public double getTargetVelocity()   { return targetVelocity; }
    public double getBatteryVoltage()   { return batteryVoltage; }
    public double getMotorPower()       { return motorPower; }
    public boolean isFeeding()          { return feeding; }
}