package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

public class Hood extends SubsystemBase {

    private final ServoEx hood;

    /*
     * Persistent suspension compression.
     *
     * 0.0 = calibrated distance position
     * Positive = lower hood position
     */
    private double retainedCorrection = 0.0;

    private double previousRpmDeficit = 0.0;
    private double filteredDeficitRate = 0.0;

    private long previousTimeNanos = System.nanoTime();

    /*
     * Lower this value for stronger compensation.
     *
     * Your old value was 1400.
     * 900 gives a considerably stronger reaction.
     */
    public static double RPM_CORRECTION_DIVISOR = 1000;

    /*
     * Extra response based on how quickly RPM drops.
     *
     * This makes the hood react strongly at the beginning of a shot,
     * before the proportional correction fully develops.
     */
    public static double RPM_DROP_RATE_GAIN = 0.000025;

    public static double RPM_DEADBAND = 20.0;

    public static double MAX_CORRECTION = 0.4;

    /*
     * Servo-position units per second.
     *
     * At 10.0, the compression can reach its requested value
     * in approximately one control loop.
     */
    public static double COMPRESSION_RATE = 12;

    /*
     * 0.30 returns a correction of 0.15 in approximately 0.5 seconds.
     */
    public static double RETURN_RATE = 0.30;

    /*
     * Larger = derivative responds more directly.
     * Smaller = derivative is smoother.
     */
    public static double RATE_FILTER_ALPHA = 0.35;

    private double lastBasePosition;
    private double lastRequestedCorrection;
    private double lastRpmDeficit;

    public Hood(HardwareMap hardwareMap) {
        hood = new ServoEx(hardwareMap, "hood");
        hood.setCachingTolerance(0.00005);
    }

    /**
     * Call continuously every TeleOp or autonomous loop.
     */
    public void run(Turret turret, Shooter shooter) {
        double dt = calculateDeltaTime();

        /*
         * Your original calibrated distance formula is unchanged.
         */
        double p =
                0.0063 * (turret.getDistanceFromBasket() - 25);

        double rpmError = shooter.getRPMError();

        /*
         * Assumes:
         *
         * rpmError = currentRPM - targetRPM
         *
         * A negative error therefore means the shooter is too slow.
         */
        double rpmDeficit = Math.max(
                0.0,
                -rpmError - RPM_DEADBAND
        );

        /*
         * Detect how quickly RPM deficit is increasing.
         */
        double rawDeficitRate =
                (rpmDeficit - previousRpmDeficit) / dt;

        previousRpmDeficit = rpmDeficit;

        filteredDeficitRate += RATE_FILTER_ALPHA
                * (rawDeficitRate - filteredDeficitRate);

        /*
         * Only use derivative when RPM is actively dropping.
         * Do not use it while RPM is recovering.
         */
        double fallingRpmRate = Math.max(
                0.0,
                filteredDeficitRate
        );

        double proportionalCorrection =
                rpmDeficit / RPM_CORRECTION_DIVISOR;

        double impactCorrection =
                fallingRpmRate * RPM_DROP_RATE_GAIN;

        double requestedCorrection = MathUtils.clamp(
                proportionalCorrection + impactCorrection,
                0.0,
                MAX_CORRECTION
        );

        if (requestedCorrection > retainedCorrection) {
            /*
             * Compress almost immediately.
             */
            retainedCorrection = moveTowards(
                    retainedCorrection,
                    requestedCorrection,
                    COMPRESSION_RATE * dt
            );
        } else {
            /*
             * Rise smoothly after RPM recovers.
             */
            retainedCorrection = moveTowards(
                    retainedCorrection,
                    requestedCorrection,
                    RETURN_RATE * dt
            );
        }

        /*
         * Keep the calibrated distance formula as the base position.
         */
        double finalPosition = MathUtils.clamp(
                p - retainedCorrection,
                0.0,
                0.8
        );

        hood.set(finalPosition);

        lastBasePosition = p;
        lastRequestedCorrection = requestedCorrection;
        lastRpmDeficit = rpmDeficit;
    }

    private double calculateDeltaTime() {
        long now = System.nanoTime();

        double dt =
                (now - previousTimeNanos) / 1_000_000_000.0;

        previousTimeNanos = now;

        if (dt < 0.005 || dt > 0.15) {
            filteredDeficitRate = 0.0;
            return 0.02;
        }

        return dt;
    }

    private double moveTowards(
            double current,
            double target,
            double maximumChange
    ) {
        if (current < target) {
            return Math.min(
                    current + maximumChange,
                    target
            );
        }

        return Math.max(
                current - maximumChange,
                target
        );
    }

    public double getRetainedCorrection() {
        return retainedCorrection;
    }

    public double getRequestedCorrection() {
        return lastRequestedCorrection;
    }

    public double getBasePosition() {
        return lastBasePosition;
    }

    public double getRpmDeficit() {
        return lastRpmDeficit;
    }

    public double getDeficitRate() {
        return filteredDeficitRate;
    }
}