package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

/**
 * Turret subsystem — CR Axon + analog encoder, custom unwrapped PID.
 *
 * ============================ FRAME CONVENTION ============================
 * The public API now speaks PHYSICAL ANGLES:
 *
 *      0   = turret pointing at ROBOT FORWARD
 *     +deg = LEFT  (counter-clockwise, same sign convention as atan2 bearings)
 *     -deg = RIGHT (clockwise)
 *
 * So a field bearing computed with atan2 minus the robot heading IS the
 * physical target — no extra "forward offset" constant to remember. The old
 * "robot angle" frame (forward = 90) is gone; the 90 is folded into the
 * calibration table below.
 *
 * WHY NOT OptimizedPositionalControl?
 * That mode always takes the SHORTEST path, which can drive the turret
 * through the back of the robot. Servo runs in RawPower; this class does the
 * control:
 *   1. UNWRAP the 0..360 analog encoder into a continuous position (with a
 *      glitch filter for the Axon's noisy seam region).
 *   2. PID error = target - unwrapped, PLAIN subtraction, never angle
 *      wrapped, so a path through the back is impossible.
 *
 * ============================ CALIBRATION ============================
 * Table rows are measured { physical deg, unwrapped servo deg } pairs,
 * sorted ascending by physical angle. To add/verify a row: point the turret
 * with dpad, read getUnwrappedServoDegrees() on telemetry, measure the real
 * physical angle with a protractor/phone, write the pair down.
 *
 * !! CONSISTENCY WARNING (check getRightHalfRatio / getLeftHalfRatio on
 * telemetry): with the current numbers the right half moves 1.50 servo deg
 * per physical deg and the left half only 1.10. A single-stage gear train
 * cannot do that — one of the labels is wrong, almost certainly the left
 * limit's "175 left". If the true ratio is 1.50 everywhere, the left hard
 * stop actually sits at (370-177)/1.50 = +129 physical, not +175. THIS IS
 * EXACTLY WHY THE TURRET STOPS SHORT (TOO FAR RIGHT) ON LEFT-SIDE TARGETS:
 * every left-half command is scaled by 1.10 instead of 1.50, i.e. ~27% of
 * the requested left rotation never happens. Re-measure the left limit's
 * physical angle and fix the row; everything else adapts automatically.
 */
@Configurable
@Config
public class Turret extends SubsystemBase {

    // ---------------- calibration ----------------
    // { PHYSICAL degrees (0 = forward, + = left), unwrapped servo degrees }
    // Sorted ascending by physical degrees!
    private static final double[][] CALIBRATION = {
            { -135.0,  -25.0 },   // right hard stop (measured)
            {    0.0,  177.0 },   // exactly forward  (measured)
            {  175.0,  380.0 },   // left hard stop   (PHYSICAL LABEL SUSPECT — likely ~129, see header)
    };

    public static final double PHYS_MIN     = CALIBRATION[0][0];
    public static final double PHYS_MAX     = CALIBRATION[CALIBRATION.length - 1][0];
    public static final double SERVO_AT_MIN = CALIBRATION[0][1];
    public static final double SERVO_AT_MAX = CALIBRATION[CALIBRATION.length - 1][1];

    // Dead zone through the back: (PHYS_MAX, PHYS_MIN + 360), center auto-computed.
    private static final double DEAD_ZONE_CENTER = (PHYS_MAX + PHYS_MIN + 360.0) / 2.0;
    private static final double HYSTERESIS  = 10.0;  // deg, prevents flip-flop at center

    private static final double SOFT_MARGIN = 4.0;   // servo deg off the hard stops
    private static final double DEADBAND    = 1.5;   // servo deg, stop jitter at target
    private static final double MAX_POWER   = 1.0;

    // Glitch filter: max believable encoder movement in ONE loop (servo deg).
    private static final double MAX_DELTA_PER_LOOP = 60.0;

    // ---------------- tunables (Dashboard friendly) ----------------
    public static double P = 0.006;
    public static double I = 0.0;
    public static double D = 0.0001;
    public static double F = 0.01;
    public static double D_FILTER = 0.8;
    public static boolean TEST = false;

    // ---------------- alliance / targets ----------------
    public enum Alliance { RED, BLUE }
    private final Pose RED_BASKET  = new Pose(138, 138);
    private final Pose BLUE_BASKET = new Pose(2, 138);
    private Alliance alliance = Alliance.BLUE;

    // ---------------- hardware ----------------
    private final CRServoEx turret;
    private final AbsoluteAnalogEncoder turretEncoder;

    // ---------------- state ----------------
    private double lastRaw;        // last ACCEPTED raw reading, 0..360
    private double unwrapped;      // continuous servo position
    private double targetServo;
    private boolean snappedToMaxSide = true;

    private double integral = 0;
    private double filteredVelocity = 0;
    private double lastPower = 0;
    private long lastTimeNs = 0;

    public Turret(HardwareMap hardwareMap) {
        turretEncoder = new AbsoluteAnalogEncoder(hardwareMap, "turretEncoder", 3.3, AngleUnit.DEGREES);
        turretEncoder.setReversed(true);
        turret = new CRServoEx(hardwareMap, "turret", turretEncoder, CRServoEx.RunMode.RawPower);
        turret.setCachingTolerance(0.002);
        turret.setInverted(false);

        // ---- initialize unwrapped position ----
        // True position is raw-360, raw, or raw+360; pick the in-range
        // candidate closest to mid-range. Correct as long as the turret
        // STARTS AWAY FROM ITS LIMITS (facing forward is ideal).
        lastRaw = readRawDegrees();
        double mid = (SERVO_AT_MIN + SERVO_AT_MAX) / 2.0;
        double best = Double.NaN;
        for (double cand : new double[]{lastRaw - 360.0, lastRaw, lastRaw + 360.0}) {
            if (cand < SERVO_AT_MIN - 10 || cand > SERVO_AT_MAX + 10) continue;
            if (Double.isNaN(best) || Math.abs(cand - mid) < Math.abs(best - mid)) best = cand;
        }
        unwrapped = Double.isNaN(best) ? lastRaw : best;
        targetServo = unwrapped;
        lastTimeNs = System.nanoTime();
    }

    // =========================================================
    //  MAIN LOOP — call turret.run() every loop() iteration
    // =========================================================
    public void run() {
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1e9;
        lastTimeNs = now;
        if (dt <= 0 || dt > 0.5) dt = 0.02;

        // ---- 1. unwrap with glitch rejection ----
        double raw = readRawDegrees();
        double delta = raw - lastRaw;
        if (delta > 180)  delta -= 360;
        if (delta < -180) delta += 360;

        if (Math.abs(delta) <= MAX_DELTA_PER_LOOP) {
            unwrapped += delta;
            lastRaw = raw;
            double instVel = delta / dt;
            filteredVelocity = D_FILTER * filteredVelocity + (1 - D_FILTER) * instVel;
        }
        // else: seam glitch — skip the sample; recovered on the next good one.

        // ---- 2. PID on unwrapped position — NO angle wrapping! ----
        double error = targetServo - unwrapped;

        double power;
        if (Math.abs(error) < DEADBAND) {
            power = 0;
            integral = 0;
        } else {
            integral += error * dt;
            power = P * error + I * integral - D * filteredVelocity + F * Math.signum(error);
            power = MathUtils.clamp(power, -MAX_POWER, MAX_POWER);
        }

        // ---- 3. hard safety ----
        if (unwrapped < SERVO_AT_MIN - 20 || unwrapped > SERVO_AT_MAX + 20) {
            power = 0;
        }

        lastPower = power;
        if (!TEST) turret.set(power);
        else       turret.set(0);
    }

    @Override
    public void periodic() { run(); }

    // =========================================================
    //  TARGETING — everything below is in PHYSICAL degrees
    // =========================================================

    public void setAlliance(Alliance alliance) { this.alliance = alliance; }
    public Alliance getAlliance() { return alliance; }
    public Pose getBasket() { return (alliance == Alliance.RED) ? RED_BASKET : BLUE_BASKET; }

    /** Auto-aim at the current alliance's basket. Requires follower.update() upstream! */
    public void aimAtBasket(Follower follower) { aimAt(getBasket(), follower); }

    /**
     * Aim at any field point. MUST use atan2 — atan(dy/dx) loses the
     * quadrant whenever dx < 0 and mirrors every left/behind target to the
     * front-right. (That is what made the turret sit right of the blue
     * basket and barely react while driving.)
     */
    public void aimAt(Pose fieldTarget, Follower follower) {
        Pose robot = follower.getPose();
        double dx = fieldTarget.getX() + robot.getX();
        double dy = fieldTarget.getY() + robot.getY();
        double fieldBearingDeg = Math.toDegrees(Math.atan2(dy, dx)); // FULL quadrant!
        double headingDeg      = Math.toDegrees(robot.getHeading());
        setTargetPhysicalAngle(fieldBearingDeg - headingDeg);       // 0 = forward, done.
    }

    /**
     * Desired turret direction in the ROBOT frame, PHYSICAL convention
     * (0 = forward, + = left / CCW). Targets inside the rear dead zone snap
     * to the nearest limit with hysteresis.
     */
    public void setTargetPhysicalAngle(double physicalDeg) {
        // normalize into [PHYS_MIN, PHYS_MIN + 360)
        double a = (physicalDeg - PHYS_MIN) % 360.0;
        if (a < 0) a += 360.0;
        a += PHYS_MIN;

        double safe;
        if (a <= PHYS_MAX) {
            safe = a;
            snappedToMaxSide = a > (DEAD_ZONE_CENTER - 180.0);
        } else {
            if (a < DEAD_ZONE_CENTER - HYSTERESIS)      snappedToMaxSide = true;
            else if (a > DEAD_ZONE_CENTER + HYSTERESIS) snappedToMaxSide = false;
            safe = snappedToMaxSide ? PHYS_MAX : PHYS_MIN;
        }

        targetServo = MathUtils.clamp(physicalToServo(safe),
                SERVO_AT_MIN + SOFT_MARGIN, SERVO_AT_MAX - SOFT_MARGIN);
    }

    /**
     * Manual nudges. LEFT = +physical = +servo (increasing servo degrees is
     * CCW in this table). The old version had these swapped.
     */
    public void moveLeft()  { nudge(+0.5); }
    public void moveRight() { nudge(-0.5); }

    private void nudge(double servoDeg) {
        targetServo = MathUtils.clamp(targetServo + servoDeg,
                SERVO_AT_MIN + SOFT_MARGIN, SERVO_AT_MAX - SOFT_MARGIN);
    }

    // =========================================================
    //  CALIBRATED CONVERSIONS (piecewise-linear over CALIBRATION)
    // =========================================================

    private double physicalToServo(double physicalDeg) { return interpolate(physicalDeg, 0, 1); }
    public  double servoToPhysical(double servoDeg)    { return interpolate(servoDeg, 1, 0); }

    private double interpolate(double x, int in, int out) {
        int n = CALIBRATION.length;
        if (x <= CALIBRATION[0][in])
            return lerpRow(x, CALIBRATION[0], CALIBRATION[1], in, out);
        if (x >= CALIBRATION[n - 1][in])
            return lerpRow(x, CALIBRATION[n - 2], CALIBRATION[n - 1], in, out);
        for (int i = 0; i < n - 1; i++) {
            if (x <= CALIBRATION[i + 1][in])
                return lerpRow(x, CALIBRATION[i], CALIBRATION[i + 1], in, out);
        }
        return CALIBRATION[n - 1][out]; // unreachable
    }

    private double lerpRow(double x, double[] a, double[] b, int in, int out) {
        double t = (x - a[in]) / (b[in] - a[in]);
        return a[out] + t * (b[out] - a[out]);
    }

    // =========================================================
    //  TELEMETRY / CALIBRATION HELPERS
    // =========================================================
    public double getCurrentPhysicalAngle()  { return servoToPhysical(unwrapped); }
    public double getTargetPhysicalAngle()   { return servoToPhysical(targetServo); }
    public double getUnwrappedServoDegrees() { return unwrapped; }
    public double getTargetServoDegrees()    { return targetServo; }
    public double getRawEncoderDegrees()     { return lastRaw; }
    public double getErrorDegrees()          { return targetServo - unwrapped; }
    public double getLastPower()             { return lastPower; }

    /** Servo deg per physical deg on each half. A healthy single-ratio
     *  gear train shows the SAME number on both. If they differ, one
     *  physical label in CALIBRATION is wrong — fix it before trusting aim. */
    public double getRightHalfRatio() {
        return (CALIBRATION[1][1] - CALIBRATION[0][1]) / (CALIBRATION[1][0] - CALIBRATION[0][0]);
    }
    public double getLeftHalfRatio() {
        int n = CALIBRATION.length;
        return (CALIBRATION[n-1][1] - CALIBRATION[n-2][1]) / (CALIBRATION[n-1][0] - CALIBRATION[n-2][0]);
    }

    private double readRawDegrees() {
        return turretEncoder.getCurrentPosition(); // absolute angle, DEGREES 0..360
    }
}