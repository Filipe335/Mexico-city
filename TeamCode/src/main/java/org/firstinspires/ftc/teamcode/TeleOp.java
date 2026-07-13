package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "main")
public class TeleOp extends OpMode {
    private Turret turret;
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;
    private Gate gate;
    private Limelight limelight;
    private double pos;
    private Timer t;
    double posX, posY;
    private Follower follower;
    private GamepadEx gamepadA, gamepadB;
    private ToggleButtonReader shooterHigh, shooterMid, shooterLow;
    @Override
    public void init() {
        gamepadA = new GamepadEx(gamepad1);
        gamepadB = new GamepadEx(gamepad2);
        shooterHigh = new ToggleButtonReader(gamepadB, GamepadKeys.Button.TRIANGLE);
        shooterMid = new ToggleButtonReader(gamepadB, GamepadKeys.Button.SQUARE);
        shooterLow = new ToggleButtonReader(gamepadB, GamepadKeys.Button.CROSS);
        turret = new Turret(hardwareMap);
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        transfer = new Transfer(hardwareMap);
        limelight = new Limelight(hardwareMap);
        t = new Timer();
        transfer.stop();
        shooter.stop();
        intake.stop();
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        turret.setAlliance(Turret.Alliance.RED);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        follower.update();
        gamepadA.readButtons();
        gamepadB.readButtons();
        shooterMid.readValue();
        shooterHigh.readValue();
        shooterLow.readValue();
//        limelight.see(follower.getHeading());
//        posX = limelight.getX();
//        posY = limelight.getY();
        //Drive:
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;

        drive.driveSmooth(x, y, rx);
        //shooter.automatic();
        //Turret:
        if (gamepad2.dpad_left)       turret.moveLeft();
        else if (gamepad2.dpad_right) turret.moveRight();
        else                          turret.aimAtBasket(follower);
        //Shooter:
        if(shooterLow.getState()){
            shooter.low();
        }else if(shooterMid.getState()){
//            shooter.override = true;
            shooter.mid();
        }else if(shooterHigh.getState()){
//            shooter.override = true;
            shooter.high();
//        }else if(gamepad2.circleWasPressed()){
//            shooter.override = false;
//            shooter.automatic();
        }else{
            shooter.stop();
        }
        if(gamepad2.shareWasPressed()){
            turret.setAlliance(Turret.Alliance.BLUE);
        }
        if(gamepad2.optionsWasPressed()){
            turret.setAlliance(Turret.Alliance.RED);
        }
        if(gamepad1.crossWasPressed()){
            follower.setPose(new Pose(127, 62, 0));
        }
        //Intake:
        if(gamepad1.right_bumper){
            intake.run();
        }else if(gamepad1.left_bumper){
            intake.reverse();
        }else{
            intake.stop();
        }
        if(gamepad1.rightTriggerWasPressed()){
            t.resetTimer();
        }
        if (gamepad1.right_trigger_pressed && t.getElapsedTimeSeconds() < 0.1) {
                gate.open();
        }else if(gamepad1.right_trigger_pressed && t.getElapsedTimeSeconds() > 0.1){
            transfer.run();
            intake.run();
        }
        if(gamepad1.rightTriggerWasReleased()){
            transfer.stop();
            gate.close();
        }
        if(gamepad1.left_trigger_pressed){
            gate.close();
            transfer.reverse();
        }
        turret.run();

        telemetry.addData("Unwrapped servo degrees ", turret.getUnwrappedServoDegrees());
        telemetry.addData("targetServoDegrees", turret.getTargetServoDegrees());
        telemetry.addData("ErrorDegrees", turret.getErrorDegrees());
        telemetry.addData("heading", follower.getHeading());
        telemetry.addData("LimelightYPos: ", posY);
        telemetry.addData("ID: ", limelight.getID());
        telemetry.addData("RPM", shooter.getRPM());
        telemetry.addData("TargetRPM", shooter.getTarget());
        telemetry.update();


    }
}
