package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;


@Configurable
@Config
public class Turret extends SubsystemBase {

    // ------ calibration ----------
    // { PHYSICAL degrees (0 = forward, + = left), unwrapped servo degrees }
    // Sorted ascending by physical degrees
    private static final double[][] CALIBRATION = {
            { -122.0,  -12 },   // right hard stop (measured)
            {    0.0,  181.0 },   // exactly forward  (measured)
            {  128.0,  355 },   // left hard stop   (PHYSICAL LABEL SUSPECT - likely 129)
    };

    public static final double PHYS_MIN     = CALIBRATION[0][0];
    public static final double PHYS_MAX     = CALIBRATION[CALIBRATION.length - 1][0];
    public static final double SERVO_AT_MIN = CALIBRATION[0][1];
    public static final double SERVO_AT_MAX = CALIBRATION[CALIBRATION.length - 1][1];
    public static  double GAMEPAD_OFFSET = 0;

    // Dead zone through the back: (PHYS_MAX, PHYS_MIN + 360), center auto-computed
    private static final double DEAD_ZONE_CENTER = (PHYS_MAX + PHYS_MIN + 360.0) / 2.0;
    private static final double HYSTERESIS  = 5.0;  // deg, prevents flip-flop at center

    private static final double SOFT_MARGIN = 10;   // servo deg off the hard stops
    private static final double DEADBAND    = 0.5;   // servo deg, stop jitter at target
    private static final double MAX_POWER   = 1.0;


    private static final double MAX_DELTA_PER_LOOP = 60;
    ///tunablees
    public static double P = 0.008;
    public static double I = 0;
    public static double D = 0.00025;
    public static double F = 0.04;
    public static double D_FILTER = 0.8;
    public static boolean TEST = false;
    public double dx, dy;

    public enum Alliance { RED, BLUE }
    private final Pose RED_BASKET  = new Pose(138, 138);
    private final Pose BLUE_BASKET = new Pose(2, 138);
    private Alliance alliance = Alliance.RED;

    private final CRServoEx turret;
    private final AbsoluteAnalogEncoder turretEncoder;
    private final ServoEx hood;

    private double lastRaw;        // last ACCEPTED raw reading, 0--360
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
        turret.setCachingTolerance(0.0002);
        turret.setInverted(false);
        hood = new ServoEx(hardwareMap, "hood");
        hood.setCachingTolerance(0.00005);

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


    public void run() {
        long now = System.nanoTime();
        double dt = (now - lastTimeNs) / 1e9;
        lastTimeNs = now;
        if (dt <= 0 || dt > 0.5) dt = 0.02;

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

        double error = targetServo - unwrapped ;

        double power;
        if (Math.abs(error) < DEADBAND) {
            power = 0;
            integral = 0;
        } else {
            integral += error * dt;
            power = P * error + I * integral - D * filteredVelocity + F * Math.signum(error);
            power = MathUtils.clamp(power, -MAX_POWER, MAX_POWER);
        }

        if (unwrapped < SERVO_AT_MIN - 20 || unwrapped > SERVO_AT_MAX + 20) {
            power = 0 ;
        }

        lastPower = power;
        if (!TEST) turret.set(power);
        else       turret.set(0);
    }

    @Override
    public void periodic() { run(); }


    public void setAlliance(Alliance alliance) { this.alliance = alliance; }
    public Alliance getAlliance() { return alliance; }
    public Pose getBasket() { return (alliance == Alliance.RED) ? RED_BASKET : BLUE_BASKET; }

    public void aimAtBasket(Follower follower) { aimAt(getBasket(), follower); }

    public void aimAt(Pose fieldTarget, Follower follower) {
        Pose robot = follower.getPose();
        dx = fieldTarget.getX() - robot.getX();
        dy = fieldTarget.getY() - robot.getY();
        double fieldBearingDeg = Math.toDegrees(Math.atan2(dy, dx)); // FULL quadrant!
        double headingDeg      = Math.toDegrees(robot.getHeading());
        setTargetPhysicalAngle(fieldBearingDeg - headingDeg + GAMEPAD_OFFSET);       // 0 = forward, done.
    }


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


    public void moveLeft()  { nudge(+0.5); }
    public void moveRight() { nudge(-0.5); }

    private void nudge(double servoDeg) {
        targetServo = MathUtils.clamp(targetServo + servoDeg,
                SERVO_AT_MIN + SOFT_MARGIN, SERVO_AT_MAX - SOFT_MARGIN) ;
    }

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


    public double getCurrentPhysicalAngle()  { return servoToPhysical(unwrapped); }
    public double getTargetPhysicalAngle()   { return servoToPhysical(targetServo); }
    public double getUnwrappedServoDegrees() { return unwrapped; }
    public double getTargetServoDegrees()    { return targetServo; }
    public double getRawEncoderDegrees()     { return lastRaw; }
    public double getErrorDegrees()          { return targetServo - unwrapped; }
    public double getLastPower()             { return lastPower; }


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
    public double getDistanceFromBasket(){
        return Math.hypot(dx, dy);
    }

    public void gamepadOffsetLeft(){
        GAMEPAD_OFFSET += 0.3;
    }
    public void gamepadOffsetRight(){
        GAMEPAD_OFFSET -= 0.3   ;
    }
    public void resetGamepadOffset(){
        GAMEPAD_OFFSET = 0;
    }

    public void gamepadMoveFreelyLeft(){
        turret.set(0.2);
    }
    public void gamepadMoveFreelyRight(){
        turret.set(-0.2);
    }
    public void gamepadStop(){
        turret.set(0);
    }
}