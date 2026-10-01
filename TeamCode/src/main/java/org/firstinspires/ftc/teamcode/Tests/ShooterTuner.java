package org.firstinspires.ftc.teamcode.Tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.ArrayList;
import java.util.List;
import java.util.Locale;

/**
 * ShooterTuner — builds the distance→RPM lookup table.
 *
 * WORKFLOW (one person drives, one tunes):
 *   1. Open FTC Dashboard. Set Shooter.sp to a guess.
 *   2. Park the robot somewhere, feed a ball, watch the shot.
 *   3. Adjust Shooter.sp until shots score reliably FROM THAT SPOT.
 *   4. Press SQUARE  -> records { distance, sp } as a table row.
 *      Press CROSS   -> deletes the last recorded row (bad point).
 *   5. Move to a new distance, repeat. 5-8 points across the range is plenty.
 *   6. Copy the printed rows into Shooter.RPM_TABLE. Done.
 *
 * BUG FIXED vs your version: distance was sqrt(dy^2 - dx^2) — minus instead
 * of plus. From your init pose (117,128) to the red basket that is
 * sqrt(100 - 441) = NaN, so the telemetry could never show a number.
 * Math.hypot(dx, dy) is the correct (and overflow-safe) form.
 */
@Config
@TeleOp(name = "ShooterTuner")
public class ShooterTuner extends OpMode {

    /** Flip from the Dashboard if you tune against the blue basket. */
    public static boolean USE_RED_BASKET = true;

    private final Pose RED_BASKET  = new Pose(138, 138);
    private final Pose BLUE_BASKET = new Pose(2, 138);

    private Shooter shooter;
    private Follower follower;

    private final List<double[]> points = new ArrayList<>();
    private boolean prevSquare = false, prevCross = false;

    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        // Heading goes IN the Pose, in RADIANS.
        follower.setStartingPose(new Pose(117, 128, Math.toRadians(36)));
        shooter = new Shooter(hardwareMap);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        follower.update();
        shooter.tune();   // setpoint = Shooter.sp (Dashboard)

        // ---- distance to goal (CORRECT formula) ----
        Pose basket = USE_RED_BASKET ? RED_BASKET : BLUE_BASKET;
        Pose robot  = follower.getPose();
        double dx = basket.getX() - robot.getX();
        double dy = basket.getY() - robot.getY();
        double distance = Math.hypot(dx, dy);   // == sqrt(dx*dx + dy*dy)

        // ---- record / undo calibration points (rising edges) ----
        boolean square = gamepad1.square;
        boolean cross  = gamepad1.cross;
        if (square && !prevSquare) {
            points.add(new double[]{ distance, Shooter.sp });
        }
        if (cross && !prevCross && !points.isEmpty()) {
            points.remove(points.size() - 1);
        }
        prevSquare = square;
        prevCross  = cross;

        // ---- live telemetry ----
        telemetry.addData("DistanceFromBasket (in)", "%.1f", distance);
        telemetry.addData("Robot X / Y", "%.1f / %.1f", robot.getX(), robot.getY());
        telemetry.addData("ShooterRPM (actual)", shooter.getRPM());
        telemetry.addData("TargetRPM (Shooter.sp)", "%.0f", Shooter.sp);
        telemetry.addData("RPM error", "%.0f", shooter.getRPMError());
        telemetry.addLine("");
        telemetry.addLine("SQUARE = record point   CROSS = undo last");
        telemetry.addLine("--- RPM_TABLE rows (paste into Shooter) ---");

        // Print sorted ascending by distance, ready to paste.
        List<double[]> sorted = new ArrayList<>(points);
        sorted.sort((a, b) -> Double.compare(a[0], b[0]));
        for (double[] p : sorted) {
            telemetry.addLine(String.format(Locale.US, "{ %6.1f, %6.0f },", p[0], p[1]));
        }
        telemetry.addData("points recorded", points.size());
        telemetry.update();
    }
}