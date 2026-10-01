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
@Autonomous(name = "RedFar", group = "Autonomous")
public class RedFar extends OpMode {
    private Timer t, intakeTimer;
    private TelemetryManager panelsTelemetry;
    private Shooter shooter;
    private Hood hood;
    private Turret turret;
    private Gate gate;
    private Transfer transfer;
    private Intake intake;
    public PathChain ShootPreload;
    public PathChain FirstLine;
    public PathChain FirstShoot;
    public PathChain PreHuman;
    public PathChain Human;
    public PathChain ShootHuman;
    public PathChain PreFirstSweep;
    public PathChain FirstSweep;
    public PathChain ShootFirstSweep;
    public PathChain PreSecondSweep;
    public PathChain SecondSweep;
    public PathChain ShootSecondSweep;
    public PathChain PreThirdSweep;
    public PathChain ThirdSweep;
    public PathChain ShootThirdSweep;
    public PathChain Park;
    private Follower follower;

    // State machine flags
    private boolean pathStarted = false;
    private boolean shootSequenceStarted = false;

    public boolean isShootingDone = false;
    public Pose startingPose = new Pose(98.000, 8.000, Math.toRadians(0));

    public enum PathState {
       SHOOT_PRELOAD,
        FIRST_LINE,
        FIRST_SHOOT,
        PRE_HUMAN,
        HUMAN,
        SHOOT_HUMAN,
        PRE_SWEEP_1,
        SWEEP_1,
        SHOOT_SWEEP_1,
        PRE_SWEEP_2,
        SWEEP_2,
        SHOOT_SWEEP_2,
        PRE_SWEEP_3,
        SWEEP_3,
        SHOOT_SWEEP_3,
        PARK
    }

    public PathState pathState;

    public void autoPathUpdate() {
        switch (pathState) {
            case SHOOT_PRELOAD:
                if(intakeTimer.getElapsedTimeSeconds() > 0.6){
                    if (!pathStarted) {
                        follower.followPath(ShootPreload);
                        pathStarted = true;
                    }
                }
                if (!follower.isBusy() && pathStarted) {
                    if (!shootSequenceStarted) {
                        t.resetTimer();
                        shootSequenceStarted = true;
                    }
                    shoot();
                }
                if(isShootingDone){
                    pathStarted = false;
                    shootSequenceStarted = false;
                    pathState = PathState.FIRST_LINE;
                }
                break;
            case FIRST_LINE:
                if(!pathStarted){
                    follower.followPath(FirstLine);
                    pathStarted = true;
                }
                intake.run();
                if(!follower.isBusy()){
                    pathStarted = false;
                    pathState = PathState.FIRST_SHOOT;
                }
                break;
            case FIRST_SHOOT:
                if(!pathStarted){
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
                        pathState = PathState.PRE_HUMAN;
                    }
                }
                break;
            case PRE_HUMAN:
                if(pathStarted){
                    follower.followPath(PreHuman);
                    pathStarted = true;
                }
                intake.run();
                if(!follower.isBusy()){
                    pathStarted = false;
                    pathState = PathState.HUMAN;
                }
                break;
            case HUMAN:
                if(!pathStarted){
                    follower.followPath(Human);
                    pathStarted = true;
                }
                intake.run();
                if(!follower.isBusy()){
                    pathStarted = false;
                    pathState = PathState.SHOOT_HUMAN;
                }
                break;
            case SHOOT_HUMAN:
                if(!pathStarted){
                    follower.followPath(ShootHuman);
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
                        pathState = PathState.PRE_SWEEP_1;
                    }
                }
                break;
            case PRE_SWEEP_1:
                if(!pathStarted){
                    follower.followPath(PreFirstSweep);
                    pathStarted = true;
                }
                intake.run();
                if(!follower.isBusy()){
                    pathStarted = false;
                    pathState = PathState.SWEEP_1;
                }
                break;
            case SWEEP_1:
                if(!pathStarted){
                    follower.followPath(FirstSweep);
                    pathStarted = true;
                }
                intake.run();
                if(!follower.isBusy()){
                    pathStarted = false;
                    pathState = PathState.SHOOT_SWEEP_1;
                }
                break;
            case SHOOT_SWEEP_1:
                if(!pathStarted){
                    follower.followPath(ShootFirstSweep);
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
                        pathState = PathState.PRE_SWEEP_2;
                    }
                }
                break;
            case PRE_SWEEP_2:
                if(!pathStarted){
                    follower.followPath(PreSecondSweep);
                    pathStarted = true;
                }
                intake.run();
                if(!follower.isBusy()){
                    pathStarted = false;
                    pathState = PathState.SWEEP_2;
                }
                break;
            case SWEEP_2:
                if(!pathStarted){
                    follower.followPath(SecondSweep);
                    pathStarted = true;
                }
                intake.run();
                if(!follower.isBusy()){
                    pathStarted = false;
                    pathState = PathState.SHOOT_SWEEP_2;
                }
                break;

            case SHOOT_SWEEP_2:
                if(!pathStarted){
                    follower.followPath(ShootSecondSweep);
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
                        pathState = PathState.PRE_SWEEP_3;
                    }
                }
                break;
            case PRE_SWEEP_3:
                if(!pathStarted){
                    follower.followPath(PreThirdSweep);
                    pathStarted = true;
                }
                intake.run();
                if(!follower.isBusy()){
                    pathStarted = false;
                    pathState = PathState.SWEEP_3;
                }
                break;
            case SWEEP_3:
                if(!pathStarted){
                    follower.followPath(ThirdSweep);
                    pathStarted = true;
                }
                intake.run();
                if(!follower.isBusy()){
                    pathStarted = false;
                    pathState = PathState.SHOOT_SWEEP_3;
                }
                break;

            case SHOOT_SWEEP_3:
                if(!pathStarted){
                    follower.followPath(ShootThirdSweep);
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

        Turret.GAMEPAD_OFFSET = 3;
        Shooter.GAMEPAD_OFFSET = 200;
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
        pathState = PathState.SHOOT_PRELOAD;
        pathStarted = false;
        shootSequenceStarted = false;
        gate.close();
        intake.stop();
        transfer.stop();
        intakeTimer.resetTimer();
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
        ShootPreload = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(98.000, 8.000),
                                new Pose(83.000, 21.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        FirstLine = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.000, 21.000),
                                new Pose(86.000, 35.000),
                                new Pose(129.000, 35.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        FirstShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(129.000, 35.000),
                                new Pose(83.000, 21.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        PreHuman = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.000, 21.000),
                                new Pose(109.000, 29.000),
                                new Pose(128.000, 17.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        Human = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(128.000, 17.000),
                                new Pose(129.000, 11.000),
                                new Pose(130.000, 9.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(0))
                .build();

        ShootHuman = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 9.000),
                                new Pose(83.000, 21.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        PreFirstSweep = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.000, 21.000),
                                new Pose(95.000, 47.000),
                                new Pose(129.000, 40.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        FirstSweep = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(129.000, 40.000),
                                new Pose(125.000, 31.000),
                                new Pose(130.000, 17.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-20))
                .build();

        ShootFirstSweep = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 17.000),
                                new Pose(83.000, 21.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();

        PreSecondSweep = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.000, 21.000),
                                new Pose(95.000, 47.000),
                                new Pose(129.000, 40.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        SecondSweep = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(129.000, 40.000),
                                new Pose(125.000, 31.000),
                                new Pose(130.000, 17.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-20))
                .build();

        ShootSecondSweep = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 17.000),
                                new Pose(83.000, 21.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();

        PreThirdSweep = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(83.000, 21.000),
                                new Pose(95.000, 47.000),
                                new Pose(129.000, 40.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-45))
                .build();

        ThirdSweep = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(129.000, 40.000),
                                new Pose(130.000, 17.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-45), Math.toRadians(-20))
                .build();

        ShootThirdSweep = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(130.000, 17.000),
                                new Pose(83.000, 21.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(83.000, 21.000),
                                new Pose(88.000, 26.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }

    public void shoot() {
        if (t.getElapsedTimeSeconds() < 0.05) {
            isShootingDone = false;
        }else if(t.getElapsedTimeSeconds() < 0.13){
            gate.open();
            isShootingDone = false;
        }else if (t.getElapsedTimeSeconds() < 1) {
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