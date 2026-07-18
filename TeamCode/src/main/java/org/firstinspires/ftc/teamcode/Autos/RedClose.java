package org.firstinspires.ftc.teamcode.Autos;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Configurable
@Autonomous(name = "RedClose", group = "Autonomous")
public class RedClose extends OpMode {
    private Timer t, intakeTimer;
    private TelemetryManager panelsTelemetry;
    private Shooter shooter;
    private Hood hood;
    private Turret turret;
    private Gate gate;
    private Transfer transfer;
    private Intake intake;
    public PathChain RunAndGun;
    public PathChain FirstPickup;
    public PathChain FirstShoot;
    public PathChain SecondPickup;
    public PathChain SecondShoot;
    public PathChain Gate1;
    public PathChain Tunnel1;
    public PathChain ShootTunnel1;
    public PathChain Gate2;
    public PathChain Tunnel2;
    public PathChain ShootTunnel2;
    public PathChain PreSweep;
    public PathChain Sweep;
    public PathChain ShootSweep;
    public PathChain Park;
    private Follower follower;

    // State machine flags
    private boolean pathStarted = false;
    private boolean shootSequenceStarted = false;

    public boolean isShootingDone = false;
    public Pose startingPose = new Pose(124.600, 124.000, Math.toRadians(33));
    public Pose turretPose = new Pose(115.10199386503068, 129.42561349693253, Math.toRadians(33));

    public enum PathState {
        RUN_AND_GUN,
        SHOOT_PRELOAD,
        FIRST_PICKUP,
        FIRST_SHOOT,
        SECOND_PICKUP,
        SECOND_SHOOT,
        GATE1,
        TUNNEL1,
        SHOOT_TUNNEL1,
        GATE2,
        TUNNEL2,
        SHOOT_TUNNEL2,
        PRE_SWEEP,
        SWEEP,
        SHOOT_SWEEP,
        PARK
    }

    public PathState pathState;

    public void autoPathUpdate() {
        switch (pathState) {
            case RUN_AND_GUN:
                if (!pathStarted) {
                    follower.followPath(RunAndGun);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = PathState.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                if (!pathStarted) {
                    t.resetTimer();
                    shootSequenceStarted = true;
                    pathStarted = true;
                }
                shoot();
                if (isShootingDone) {
                    pathStarted = false;
                    shootSequenceStarted = false;
                    pathState = PathState.FIRST_PICKUP;
                }
                break;

            case FIRST_PICKUP:
                if (!pathStarted) {
                    follower.followPath(FirstPickup);
                    pathStarted = true;
                }
                intake.run();
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = PathState.FIRST_SHOOT;
                }
                break;

            case FIRST_SHOOT:
                if (!pathStarted) {
                    follower.followPath(FirstShoot);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    if (!shootSequenceStarted) {
                        t.resetTimer();
                        shootSequenceStarted = true;
                    }
                    shoot();
                    if (isShootingDone) {
                        pathStarted = false;
                        shootSequenceStarted = false;
                        pathState = PathState.SECOND_PICKUP;
                    }
                }
                break;

            case SECOND_PICKUP:
                if (!pathStarted) {
                    follower.followPath(SecondPickup);
                    pathStarted = true;
                }
                intake.run();
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = PathState.SECOND_SHOOT;
                }
                break;

            case SECOND_SHOOT:
                if (!pathStarted) {
                    follower.followPath(SecondShoot);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    if (!shootSequenceStarted) {
                        t.resetTimer();
                        shootSequenceStarted = true;
                    }
                    shoot();
                    if (isShootingDone) {
                        pathStarted = false;
                        shootSequenceStarted = false;
                        pathState = PathState.GATE1;
                    }
                }
                break;

            case GATE1:
                if (!pathStarted) {
                    follower.followPath(Gate1);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = PathState.TUNNEL1;
                }
                break;

            case TUNNEL1:
                if (!pathStarted) {
                    follower.followPath(Tunnel1);
                    intakeTimer.resetTimer();
                    pathStarted = true;
                }
                intake.run();
                if (!follower.isBusy()) {
                    // Wait 1.2 seconds after the path finishes to allow intake from gate
                    if (intakeTimer.getElapsedTimeSeconds() < 1.2) {
                        intake.run();
                    } else {
                        pathStarted = false;
                        pathState = PathState.SHOOT_TUNNEL1;
                    }
                }
                break;

            case SHOOT_TUNNEL1:
                if (!pathStarted) {
                    follower.followPath(ShootTunnel1);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    if (!shootSequenceStarted) {
                        t.resetTimer();
                        shootSequenceStarted = true;
                    }
                    shoot();
                    if (isShootingDone) {
                        pathStarted = false;
                        shootSequenceStarted = false;
                        pathState = PathState.GATE2;
                    }
                }
                break;

            case GATE2:
                if (!pathStarted) {
                    follower.followPath(Gate2);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = PathState.TUNNEL2;
                }
                break;

            case TUNNEL2:
                if (!pathStarted) {
                    follower.followPath(Tunnel2);
                    intakeTimer.resetTimer();
                    pathStarted = true;
                }
                intake.run();
                if (!follower.isBusy()) {
                    // Wait 1.2 seconds after the path finishes to allow intake from gate
                    if (intakeTimer.getElapsedTimeSeconds() < 1.2) {
                        intake.run();
                    } else {
                        pathStarted = false;
                        pathState = PathState.SHOOT_TUNNEL2;
                    }
                }
                break;

            case SHOOT_TUNNEL2:
                if (!pathStarted) {
                    follower.followPath(ShootTunnel2);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    if (!shootSequenceStarted) {
                        t.resetTimer();
                        shootSequenceStarted = true;
                    }
                    shoot();
                    if (isShootingDone) {
                        pathStarted = false;
                        shootSequenceStarted = false;
                        pathState = PathState.PRE_SWEEP;
                    }
                }
                break;

            case PRE_SWEEP:
                if (!pathStarted) {
                    follower.followPath(PreSweep);
                    pathStarted = true;
                }
                intake.run();
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = PathState.SWEEP;
                }
                break;

            case SWEEP:
                if (!pathStarted) {
                    follower.followPath(Sweep);
                    pathStarted = true;
                }
                intake.run();
                if (!follower.isBusy()) {
                    pathStarted = false;
                    pathState = PathState.SHOOT_SWEEP;
                }
                break;

            case SHOOT_SWEEP:
                if (!pathStarted) {
                    follower.followPath(ShootSweep);
                    pathStarted = true;
                }
                if (!follower.isBusy()) {
                    if (!shootSequenceStarted) {
                        t.resetTimer();
                        shootSequenceStarted = true;
                    }
                    shoot();
                    if (isShootingDone) {
                        pathStarted = false;
                        shootSequenceStarted = false;
                        pathState = PathState.PARK;
                    }
                }
                break;

            case PARK:
                if (!pathStarted) {
                    follower.followPath(Park);
                    pathStarted = true;
                }
                panelsTelemetry.debug("Auto is done");
                break;
        }
    }

    @Override
    public void init() {
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        shooter = new Shooter(hardwareMap);
        turret = new Turret(hardwareMap);
        gate = new Gate(hardwareMap);
        transfer = new Transfer(hardwareMap);
        intake = new Intake(hardwareMap);
        hood = new Hood(hardwareMap);
        turret.setAlliance(Turret.Alliance.RED);

        Turret.GAMEPAD_OFFSET = 8;
        Shooter.GAMEPAD_OFFSET = 50;
        t = new Timer();
        intakeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        buildPaths(follower);

        panelsTelemetry.debug("INIT FINISHED");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void start() {
        pathState = PathState.RUN_AND_GUN;
        pathStarted = false;
        shootSequenceStarted = false;
    }

    @Override
    public void loop() {
        follower.update();
        turret.aimAtBasket(follower);
        turret.run();
        hood.run(turret, shooter);
        shooter.setFromDistance(turret.getDistanceFromBasket());

        autoPathUpdate();
        telemetry.addData("Unwrapped servo degrees ", turret.getUnwrappedServoDegrees());
        telemetry.addData("targetServoDegrees", turret.getTargetServoDegrees());
        telemetry.addData("ErrorDegrees", turret.getErrorDegrees());
        telemetry.addData("TargetPhysicalAngle", turret.getTargetPhysicalAngle());
        telemetry.addData("PhysicalAngle", turret.getCurrentPhysicalAngle());
        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }

    public void buildPaths(Follower follower) {
        RunAndGun = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(124.600, 124.000), new Pose(91.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(34), Math.toRadians(0))
                .build();
        FirstPickup = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(91.000, 83.000), new Pose(123.000, 82.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        FirstShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(123.000, 82.000), new Pose(91.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        SecondShoot = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(85, 60.000), new Pose(91.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        Gate1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(91.000, 83.000), new Pose(124, 63)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Tunnel1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(124.000, 63.000),
                                new Pose(123.000, 56.000),
                                new Pose(132, 51.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
        ShootTunnel1 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(132, 51), new Pose(91.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
        Gate2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(91.000, 83.000), new Pose(124.000, 63.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        Tunnel2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(124.000, 63.000),
                                new Pose(123.000, 56.000),
                                new Pose(132, 51.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(45))
                .build();
        ShootTunnel2 = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(132, 51.000), new Pose(91.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(45), Math.toRadians(0))
                .build();
        PreSweep = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(91.000, 83.000), new Pose(127.000, 57)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();
        Sweep = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(127.000, 57), new Pose(129.000, 40.000), new Pose(129.000, 40.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-20))
                .build();
        ShootSweep = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(129, 40), new Pose(91.000, 83.000)))
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();
        Park = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(91.000, 83.000), new Pose(111.000, 79.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
        SecondPickup = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(91.000, 83.000), new Pose(85.000, 60.000), new Pose(126.000, 60.000)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public void shoot() {
        if (t.getElapsedTimeSeconds() < 0.35) {
            gate.open();
            isShootingDone = false;
        } else if (t.getElapsedTimeSeconds() < 0.9) {
            intake.run();
            transfer.run();
            isShootingDone = false;
        } else {
            gate.close();
            intake.stop();
            transfer.stop();
            isShootingDone = true;
        }
    }
}