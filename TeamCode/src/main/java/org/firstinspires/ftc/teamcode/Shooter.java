package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

/**
 * Shooter — dual flywheel motors, one velocity PIDF, distance→RPM lookup table.
 *
 * FIXES vs old version:
 *  1. ticksToRPM used int math: "-(int)ticks * 31/14" truncates BEFORE the
 *     ratio and rounds the ratio itself. Now pure double math. (Verify the
 *     31/14 factor against your encoder CPR — it is kept as-is from your code.)
 *  2. pid.calculate() was called TWICE per loop (once per motor). The second
 *     call sees dt ≈ 0 and double-steps the integral, so I was effectively 2x
 *     and D was garbage. Now: calculate ONCE, apply the same output to both.
 *  3. The InterpLUT was created in the constructor and thrown away. It is now
 *     a real field, filled from RPM_TABLE, with a setFromDistance() API.
 *
 * BUILDING THE TABLE: run the ShooterTuner OpMode. It shows live distance and
 * RPM, lets you tune Shooter.sp from the Dashboard until shots score, and on
 * a button press records the (distance, target RPM) pair, printing ready-to-
 * paste rows for RPM_TABLE below.
 */
@Config
@Configurable
public class Shooter extends SubsystemBase {

    // =========================================================
    //  DISTANCE → RPM LOOKUP TABLE  (from ShooterTuner sessions)
    //  { distance to goal (inches), flywheel RPM }
    //  MUST be sorted ascending by distance, minimum 2 rows.
    //  Replace these seed rows with your measured pairs!
    // =========================================================
    public static double[][] RPM_TABLE = {
            //  distance,  rpm     <- paste ShooterTuner output here
            {   30.0,     1000 },
            {  140.0,     4000 },
    };

    private final InterpLUT rpmForDistance = new InterpLUT();
    private final double lutMin, lutMax;

    // ---------------- tunables ----------------
    public static double P = 0.004;
    public static double I = 0.0001;
    public static double D = 0.0001;
    public static double F = 0.000195;
    public static double sp = 0;          // ShooterTuner setpoint (Dashboard)

    public static PIDFController pid;
    public boolean override;

    private final MotorEx shooter;
    private final MotorEx shooter2;
    private final ServoEx hood;

    private double targetRPM = 0;

    public Shooter(HardwareMap hardwareMap) {
        shooter  = new MotorEx(hardwareMap, "shooter");
        shooter2 = new MotorEx(hardwareMap, "shooter2");
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter2.setRunMode(Motor.RunMode.RawPower);
        shooter2.setInverted(true);

        pid = new PIDFController(P, I, D, F);

        hood = new ServoEx(hardwareMap, "hood");
        hood.setCachingTolerance(0.00005);

        // ---- build the LUT from RPM_TABLE ----
        for (double[] row : RPM_TABLE) rpmForDistance.add(row[0], row[1]);
        rpmForDistance.createLUT();
        lutMin = RPM_TABLE[0][0];
        lutMax = RPM_TABLE[RPM_TABLE.length - 1][0];
    }

    // =========================================================
    //  DISTANCE-BASED SHOOTING (the point of the LUT)
    // =========================================================

    /** RPM the table prescribes for a given distance (inches). Input is
     *  clamped to the calibrated range — InterpLUT throws outside it. */
    public double rpmForDistance(double distanceInches) {
        double d = Math.max(lutMin + 1e-6, Math.min(lutMax - 1e-6, distanceInches));
        return rpmForDistance.get(d);
    }

    /** Spin up to the RPM the lookup table prescribes for this distance. */
    public void setFromDistance(double distanceInches) {
        runTo(rpmForDistance(distanceInches));
    }

    // =========================================================
    //  FIXED PRESETS
    // =========================================================
    public void low()  { runTo(1000); }
    public void mid()  { runTo(2800); }
    public void high() { runTo(4000); }

    /** ShooterTuner mode: setpoint = Shooter.sp from the Dashboard. */
    public void tune() { runTo(sp); }

    public void stop() {
        targetRPM = 0;
        shooter.set(0);
        shooter2.set(0);
    }

    /** ONE pid.calculate per loop; same output to both coupled motors. */
    private void runTo(double rpm) {
        targetRPM = rpm;
        pid.setCoefficients(new PIDFCoefficients(P, I, D, F));
        double out = pid.calculate(getRPMExact(), rpm);
        shooter.set(out);
        shooter2.set(out);
    }

    // =========================================================
    //  MEASUREMENT / TELEMETRY
    // =========================================================

    /** ticks/s → RPM in double math. VERIFY the 31/14 factor: it should be
     *  60 * gearRatio / encoderCPR for your motor. */
    public double ticksToRPM(double ticksPerSec) {
        return -ticksPerSec * 31.0 / 14.0;
    }

    public double getRPMExact() { return ticksToRPM(shooter2.getVelocity()); }
    public int    getRPM()      { return (int) Math.round(getRPMExact()); }
    public double getTarget()   { return targetRPM; }

    /** |target - actual| — handy for a "ready to shoot" indicator. */
    public double getRPMError() { return targetRPM - getRPMExact(); }
    public boolean atSpeed(double toleranceRPM) {
        return targetRPM > 0 && Math.abs(getRPMError()) < toleranceRPM;
    }
}