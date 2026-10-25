package org.firstinspires.ftc.teamcode.sim.Util;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import org.codeblooded.ftcodesim.hardware.devices.SimHardwareMechanism;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;

/**
 * Lets FTCodeSim run LinearOpModes (like StarterBot's Teleop).
 *
 * Why this is needed: FTCodeSim drives an OpMode by calling init(), init_loop(),
 * start(), loop() and stop(). In the FTC SDK (9.0 and later) those methods are
 * final and empty in LinearOpMode; the real Robot Controller runs runOpMode()
 * through internal hooks instead. So in plain FTCodeSim a LinearOpMode's code
 * never runs.
 *
 * What this does, every physics tick:
 *   - when FTCodeSim INITs a LinearOpMode, it starts runOpMode() on its own thread
 *     (just like the Robot Controller does)
 *   - when you press PLAY it sets the SDK's internal "isStarted" flag and wakes
 *     waitForStart(); when you press STOP it sets "stopRequested" and interrupts
 *     the thread, so opModeIsActive() goes false and the OpMode ends
 * The flag names are the same ones PsiKit uses for its FTC replay support.
 *
 * Usage in the sim test:
 * <pre>
 *     LinearOpModeSupport linear = new LinearOpModeSupport();
 *     simHardwareMap.register(linear);
 *     FTCodeSim sim = new FTCodeSim(simConfig);
 *     linear.attach(sim);
 *     sim.run();
 * </pre>
 */
public class LinearOpModeSupport implements SimHardwareMechanism {
    private Object sim;
    private Object currentLifecycle;
    private LinearOpMode opMode;
    private Thread thread;
    private boolean startSent, stopSent;

    /** Give it the FTCodeSim instance (after constructing it, before sim.run()). */
    public void attach(Object ftcodesim) {
        this.sim = ftcodesim;
    }

    @Override
    public void update(double deltaTime) {
        if (sim == null) return;
        Object lifecycle = read(sim, "opModeLifecycle");

        if (lifecycle != currentLifecycle) {
            finish();
            currentLifecycle = lifecycle;
            Object om = lifecycle == null ? null : read(lifecycle, "opMode");
            if (om instanceof LinearOpMode) launch((LinearOpMode) om);
        }
        if (thread == null || lifecycle == null) return;

        boolean started = Boolean.TRUE.equals(read(lifecycle, "isStarted"));
        boolean stopped = Boolean.TRUE.equals(read(lifecycle, "isStopped"));

        if (started && !startSent) {
            setFlag(opMode, "isStarted", true);
            startSent = true;
        }
        if (startSent && thread.isAlive()) wake(opMode);   // waitForStart() may be parked on a notifier
        if (stopped && !stopSent) {
            finish();
        }
    }

    @Override
    public void reset() {
        finish();
        currentLifecycle = null;
    }

    private void launch(LinearOpMode om) {
        opMode = om;
        startSent = false;
        stopSent = false;
        // FTCodeSim reuses one instance per OpMode class, so clear the last run's state
        setFlag(om, "isStarted", false);
        setFlag(om, "stopRequested", false);
        String name = om.getClass().getSimpleName();
        thread = new Thread(() -> {
            try {
                om.runOpMode();
            } catch (InterruptedException e) {
                // normal STOP
            } catch (Throwable t) {
                System.out.println("[Sim] " + name + " crashed:");
                t.printStackTrace(System.out);
            }
            System.out.println("[Sim] " + name + " runOpMode() finished");
        }, "LinearOpMode " + name);
        thread.setDaemon(true);
        thread.start();
        System.out.println("[Sim] Running LinearOpMode " + name + " on its own thread");
    }

    /** Ask the running LinearOpMode to stop and wait (briefly) for it. */
    private void finish() {
        if (thread == null) return;
        stopSent = true;
        setFlag(opMode, "stopRequested", true);
        thread.interrupt();
        wake(opMode);
        try {
            thread.join(1000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        if (thread.isAlive()) {
            System.out.println("[Sim] " + opMode.getClass().getSimpleName()
                    + " did not stop within 1 s: check its loops use opModeIsActive()");
        }
        thread = null;
    }

    // ------------------------------------------------------------ reflection helpers

    private static Object read(Object target, String name) {
        Field f = find(target.getClass(), name);
        if (f == null) return null;
        try {
            return f.get(target);
        } catch (IllegalAccessException e) {
            return null;
        }
    }

    private static void setFlag(Object target, String name, boolean value) {
        Field f = find(target.getClass(), name);
        if (f == null || f.getType() != boolean.class) {
            System.out.println("[Sim] Warning: SDK field '" + name + "' not found; LinearOpMode start/stop may not work");
            return;
        }
        try {
            f.setBoolean(target, value);
        } catch (IllegalAccessException ignored) {
        }
    }

    /** notifyAll() on every plain lock object the SDK's LinearOpMode/OpMode classes hold. */
    private static void wake(Object target) {
        for (Class<?> c = target.getClass(); c != null && c != Object.class; c = c.getSuperclass()) {
            if (!c.getName().startsWith("com.qualcomm.robotcore")) continue;
            for (Field f : c.getDeclaredFields()) {
                if (f.getType() != Object.class || Modifier.isStatic(f.getModifiers())) continue;
                try {
                    f.setAccessible(true);
                    Object lock = f.get(target);
                    if (lock != null) {
                        synchronized (lock) {
                            lock.notifyAll();
                        }
                    }
                } catch (Exception ignored) {
                }
            }
        }
    }

    private static final java.util.Map<String, Field> FIELDS = new java.util.concurrent.ConcurrentHashMap<>();

    private static Field find(Class<?> start, String name) {
        String key = start.getName() + "#" + name;
        Field cached = FIELDS.get(key);
        if (cached != null) return cached;
        Field f = findUncached(start, name);
        if (f != null) FIELDS.put(key, f);
        return f;
    }

    private static Field findUncached(Class<?> c, String name) {
        for (; c != null; c = c.getSuperclass()) {
            try {
                Field f = c.getDeclaredField(name);
                f.setAccessible(true);
                return f;
            } catch (NoSuchFieldException ignored) {
            } catch (RuntimeException e) {
                return null;
            }
        }
        return null;
    }
}
