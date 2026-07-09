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

import org.firstinspires.ftc.teamcode.Gate;
import org.firstinspires.ftc.teamcode.Intake;
import org.firstinspires.ftc.teamcode.Shooter;
import org.firstinspires.ftc.teamcode.Transfer;
import org.firstinspires.ftc.teamcode.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
@Configurable
@Autonomous(name = "RedClose", group = "Autonomous")
public class RedClose extends OpMode {
    private Timer t;
    private TelemetryManager panelsTelemetry;
    private Shooter shooter;
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
    public Pose startingPose = new Pose(117.000, 128.000);
    public enum PathState{
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
        SWEEP_SHOOT,
        PARK
    }
    public PathState pathState;
    public void autoPathUpdate(){
        //TODO: Implement Turret logic
        switch(pathState){
            case RUN_AND_GUN:
                follower.followPath(RunAndGun);
                if(!follower.isBusy()){
                    pathState = PathState.SHOOT_PRELOAD;
                }
                break;

            case SHOOT_PRELOAD:
                t.resetTimer();
                if(!follower.isBusy()){
                    shoot();
                }
                transfer.stop();
                gate.close();
                pathState = PathState.FIRST_PICKUP;
                break;

            case FIRST_PICKUP:
                follower.followPath(FirstPickup);
                intake.run();
                if(!follower.isBusy()){
                    intake.stop();
                    pathState = PathState.FIRST_SHOOT;
                }
                break;

            case FIRST_SHOOT:
                follower.followPath(FirstShoot);
                if(!follower.isBusy()){
                    shoot();
                }
                pathState = PathState.SECOND_PICKUP;
                break;
            case SECOND_PICKUP:
                follower.followPath(SecondPickup);
                intake.run();
                if(follower.atPose(SecondPickup.endPose(), 1, 1)){
                    intake.stop();
                    pathState = PathState.SECOND_SHOOT;
                }
                break;
            case SECOND_SHOOT:
                follower.followPath(SecondShoot);
                if(!follower.isBusy()){
                    shoot();
                }
                pathState = PathState.GATE1;
                break;
            case GATE1:
                follower.followPath(Gate1);
                if (follower.atPose(Gate1.endPose(), 1, 1)){
                    pathState = PathState.TUNNEL1;
                }
                break;
            case TUNNEL1:
                intake.run();
                follower.followPath(Tunnel1);
                if(follower.atPose(Tunnel1.endPose(), 1, 1)){
                    intake.stop();
                    pathState = PathState.SHOOT_TUNNEL1;
                }
                break;
            case SHOOT_TUNNEL1:
                if(follower.atPose(Tunnel1.endPose(), 0.5, 0.5)){
                    shoot();
                }
                pathState = PathState.GATE2;
                break;
            case GATE2:
                follower.followPath(Gate2);
                if(follower.atPose(Gate2.endPose(), 1, 1)){
                    pathState = PathState.TUNNEL2;
                }
                break;
            case TUNNEL2:
                intake.run();
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

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        buildPaths(follower);

        panelsTelemetry.debug("INIT FINISHED");
        panelsTelemetry.update(telemetry);
    }
    @Override
    public void loop() {
        follower.update();
        autoPathUpdate();
        //Always on shooter -> changeable
        shooter.mid();

        panelsTelemetry.debug("Path State", pathState);
        panelsTelemetry.debug("X", follower.getPose().getX());
        panelsTelemetry.debug("Y", follower.getPose().getY());
        panelsTelemetry.debug("Heading", follower.getPose().getHeading());
        panelsTelemetry.update(telemetry);
    }
    public void buildPaths(Follower follower) {
        //TODO
        RunAndGun = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(117.000, 128.000),
                                new Pose(95.000, 81.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        FirstPickup = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(95.000, 81.000),
                                new Pose(121.000, 82.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        FirstShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(121.000, 82.000),
                                new Pose(95.000, 81.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        SecondPickup = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(95.000, 81.000),
                                new Pose(96.000, 57.000),
                                new Pose(120.000, 59.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        SecondShoot = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(120.000, 59.000),
                                new Pose(95.000, 81.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Gate1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(95.000, 81.000),
                                new Pose(125.000, 64.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Tunnel1 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(125.000, 64.000),
                                new Pose(118.000, 52.000),
                                new Pose(131.000, 49.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        ShootTunnel1 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(131.000, 49.000),
                                new Pose(95.000, 81.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

        Gate2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(95.000, 81.000),
                                new Pose(125.000, 64.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        Tunnel2 = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(125.000, 64.000),
                                new Pose(118.000, 52.000),
                                new Pose(131.000, 49.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(40))
                .build();

        ShootTunnel2 = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(131.000, 49.000),
                                new Pose(95.000, 81.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(40), Math.toRadians(0))
                .build();

        PreSweep = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(95.000, 81.000),
                                new Pose(107.000, 62.000),
                                new Pose(128.000, 57.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(-50))
                .build();

        Sweep = follower.pathBuilder()
                .addPath(
                        new BezierCurve(
                                new Pose(128.000, 57.000),
                                new Pose(126.000, 52.000),
                                new Pose(131.000, 43.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-50), Math.toRadians(-20))
                .build();

        ShootSweep = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(131.000, 43.000),
                                new Pose(95.000, 81.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(-20), Math.toRadians(0))
                .build();

        Park = follower.pathBuilder()
                .addPath(
                        new BezierLine(
                                new Pose(95.000, 81.000),
                                new Pose(103.000, 81.000)
                        )
                )
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();
    }
    public void shoot(){
        if(t.getElapsedTimeSeconds() < 0.06){
            gate.open();
        }else if(t.getElapsedTimeSeconds() < 0.4){
            intake.run();
            transfer.run();
        }else {
            gate.close();
            intake.stop();
            transfer.stop();
        }
    }
}


