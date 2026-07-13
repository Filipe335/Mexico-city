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
 * WHY NOT OptimizedPositionalControl?
 * That mode always takes the SHORTEST path, which can drive the turret through
 * the back of the robot. Servo runs in RawPower; this class does the control:
 *   1. UNWRAP the 0..360 analog encoder into a continuous position by watching
 *      for jumps > 180 between loops (with a glitch filter for the Axon's
 *      noisy seam region).
 *   2. PID error = target - unwrapped, PLAIN subtraction, never angle-wrapped,
 *      so a path through the back is impossible. Target flips between limits
 *      produce the long swing around the FRONT automatically.
 *
 * CALIBRATION TABLE — measured {robot deg, unwrapped servo deg} pairs.
 * Piecewise-linear interpolation between rows. MUST be sorted ascending.
 * Add more rows any time you measure a new pair (e.g. robot 0 and 180):
 * point the turret with dpad, read getUnwrappedServoDegrees() on telemetry,
 * and write down the pair.
 *
 * NOTE: current data is NOT linear (1.53 servo/robot deg on the right half,
 * 1.19 on the left half). If the gearing is constant, the "265" left limit
 * label is probably wrong (true value likely ~226). Verify with the tests,
 * then fix the table — the code adapts automatically, including the dead
 * zone center.
 */
@Configurable
@Config
public class Turret extends SubsystemBase {

    // ---------------- calibration ----------------
    // { robot degrees, unwrapped servo degrees } — sorted by robot degrees!
    private static final double[][] CALIBRATION = {
            { -45.0,  -25.0 },   // right limit (measured)
            {  90.0,  177.0 },   // facing exactly forward (measured)
            { 265.0,  370.0 },   // left limit (measured — robot angle label suspect!)
    };
    private static final double TURRET_FORWARD_OFFSET = 90.0;  // from CALIBRATION: robot 90 = forward

    public static final double ROBOT_MIN = CALIBRATION[0][0];
    public static final double ROBOT_MAX = CALIBRATION[CALIBRATION.length - 1][0];
    public static final double SERVO_AT_MIN = CALIBRATION[0][1];
    public static final double SERVO_AT_MAX = CALIBRATION[CALIBRATION.length - 1][1];

    // Dead zone through the back: (ROBOT_MAX, ROBOT_MIN+360), center auto-computed.
    private static final double DEAD_ZONE_CENTER = (ROBOT_MAX + ROBOT_MIN + 360.0) / 2.0;
    private static final double HYSTERESIS = 10.0;   // deg, prevents flip-flop at center

    private static final double SOFT_MARGIN = 4.0;   // servo deg off the hard stops
    private static final double DEADBAND = 1.5;      // servo deg, stop jitter at target
    private static final double MAX_POWER = 1.0;

    // Glitch filter: max believable encoder movement in ONE loop (servo deg).
    // Axon tops out around ~600 deg/s; at 50 Hz that's ~12 deg/loop. 60 is a
    // generous ceiling — anything bigger is treated as a seam glitch and skipped.
    private static final double MAX_DELTA_PER_LOOP = 60.0;

    // ---------------- tunables (FTC Dashboard friendly) ----------------
    public static double P = 0.006;
    public static double I = 0.0;
    public static double D = 0.0001;      // start at 0! add back slowly after testing
    public static double F = 0.01;     // static-friction feedforward: F * signum(error)
    public static double D_FILTER = 0.8; // 0..1, higher = smoother derivative

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
    private double lastRaw;            // last ACCEPTED raw reading, 0..360
    private double unwrapped;          // continuous servo position
    private double targetServo;
    private boolean snappedToMaxSide = true;

    private double integral = 0;
    private double filteredVelocity = 0;  // low-passed deg/s, for derivative-on-measurement
    private double lastPower = 0;
    private long lastTimeNs = 0;

    public Turret(HardwareMap hardwareMap) {
        turretEncoder = new AbsoluteAnalogEncoder(hardwareMap, "turretEncoder", 3.3, AngleUnit.DEGREES);
        turretEncoder.setReversed(true);
        turret = new CRServoEx(hardwareMap, "turret", turretEncoder, CRServoEx.RunMode.RawPower);
        turret.setCachingTolerance(0.002);
        turret.setInverted(false);

        // ---- initialize unwrapped position ----
        // The true position is raw-360, raw, or raw+360; pick the candidate in
        // range closest to mid-range. Correct as long as the turret STARTS AWAY
        // FROM ITS LIMITS (facing forward is ideal).
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
            // low-pass filtered velocity (deg/s) for the D term
            double instVel = delta / dt;
            filteredVelocity = D_FILTER * filteredVelocity + (1 - D_FILTER) * instVel;
        }
        // else: seam glitch — skip this sample entirely. lastRaw stays at the
        // last GOOD reading, so real motion is recovered on the next good one.

        // ---- 2. PID on unwrapped position — NO angle wrapping! ----
        double error = targetServo - unwrapped;

        double power;
        if (Math.abs(error) < DEADBAND) {
            power = 0;
            integral = 0;
        } else {
            integral += error * dt;
            // derivative on (filtered) measurement: -D * velocity. Damps motion
            // without amplifying setpoint changes or raw sensor spikes.
            power = P * error + I * integral - D * filteredVelocity + F * Math.signum(error);
            power = MathUtils.clamp(power, -MAX_POWER, MAX_POWER);
        }

        // ---- 3. hard safety ----
        if (unwrapped < SERVO_AT_MIN - 20 || unwrapped > SERVO_AT_MAX + 20) {
            power = 0;
        }

        lastPower = power;

        if(!TEST) turret.set(power);
        else turret.set(0);
    }

    @Override
    public void periodic() { run(); }   // command-based support

    // =========================================================
    //  TARGETING
    // =========================================================

    public void setAlliance(Alliance alliance) { this.alliance = alliance; }
    public Alliance getAlliance() { return alliance; }
    public Pose getBasket() { return (alliance == Alliance.RED) ? RED_BASKET : BLUE_BASKET; }

    /** Auto-aim at the current alliance's basket. Requires follower.update() upstream! */
    public void aimAtBasket(Follower follower) { aimAt(getBasket(), follower); }

    public void aimAt(Pose fieldTarget, Follower follower) {
        double dx = fieldTarget.getX() - follower.getPose().getX();
        double dy = fieldTarget.getY() - follower.getPose().getY();
        double fieldAngleDeg = Math.toDegrees(Math.atan(dy/dx));
        double robotHeadingDeg = Math.toDegrees(follower.getHeading());
        setTargetRobotAngle(fieldAngleDeg -robotHeadingDeg + TURRET_FORWARD_OFFSET);
    }

    /** Desired turret heading in ROBOT frame; dead-zone targets snap to a limit. */
    public void setTargetRobotAngle(double robotAngle) {
        // normalize into [ROBOT_MIN, ROBOT_MIN + 360)
        double a = (robotAngle - ROBOT_MIN) % 360.0;
        if (a < 0) a += 360.0;
        a += ROBOT_MIN;

        double safe;
        if (a <= ROBOT_MAX) {
            safe = a;
            snappedToMaxSide = a > (DEAD_ZONE_CENTER - 180.0);
        } else {
            if (a < DEAD_ZONE_CENTER - HYSTERESIS)       snappedToMaxSide = true;
            else if (a > DEAD_ZONE_CENTER + HYSTERESIS)  snappedToMaxSide = false;
            safe = snappedToMaxSide ? ROBOT_MAX : ROBOT_MIN;
        }

        targetServo = MathUtils.clamp(robotToServo(safe),
                SERVO_AT_MIN + SOFT_MARGIN, SERVO_AT_MAX - SOFT_MARGIN);
    }

    /** Manual nudges — 0.5 servo deg per call. */
    public void moveLeft()  { nudge(-0.5); }
    public void moveRight() { nudge(+0.5); }

    private void nudge(double servoDeg) {
        targetServo = MathUtils.clamp(targetServo + servoDeg,
                SERVO_AT_MIN + SOFT_MARGIN, SERVO_AT_MAX - SOFT_MARGIN);
    }

    // =========================================================
    //  CALIBRATED CONVERSIONS (piecewise-linear interpolation)
    // =========================================================

    private double robotToServo(double robotAngle) {
        return interpolate(robotAngle, 0, 1);
    }

    public double servoToRobot(double servoDeg) {
        return interpolate(servoDeg, 1, 0);
    }

    /** Piecewise-linear interpolation over CALIBRATION, from column `in` to column `out`. */
    private double interpolate(double x, int in, int out) {
        int n = CALIBRATION.length;
        if (x <= CALIBRATION[0][in])
            return extrapolate(x, CALIBRATION[0], CALIBRATION[1], in, out);
        if (x >= CALIBRATION[n - 1][in])
            return extrapolate(x, CALIBRATION[n - 2], CALIBRATION[n - 1], in, out);
        for (int i = 0; i < n - 1; i++) {
            if (x <= CALIBRATION[i + 1][in])
                return extrapolate(x, CALIBRATION[i], CALIBRATION[i + 1], in, out);
        }
        return CALIBRATION[n - 1][out]; // unreachable
    }

    private double extrapolate(double x, double[] a, double[] b, int in, int out) {
        double t = (x - a[in]) / (b[in] - a[in]);
        return a[out] + t * (b[out] - a[out]);
    }

    // =========================================================
    //  TELEMETRY
    // =========================================================
    public double getCurrentRobotAngle()     { return servoToRobot(unwrapped); }
    public double getUnwrappedServoDegrees() { return unwrapped; }
    public double getTargetServoDegrees()    { return targetServo; }
    public double getRawEncoderDegrees()     { return lastRaw; }
    public double getErrorDegrees()          { return targetServo - unwrapped; }
    public double getLastPower()             { return lastPower; }

    private double readRawDegrees() {
        return turretEncoder.getCurrentPosition(); // absolute angle, DEGREES 0..360
    }
}