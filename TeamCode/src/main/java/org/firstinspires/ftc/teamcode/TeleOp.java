package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

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
    double x, y, rx;
    double posX, posY;
    private Follower follower;
    @Override
    public void init() {
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
//        follower = Constants.createFollower(hardwareMap);

    }

    @Override
    public void loop() {
//        limelight.see(follower.getHeading());
//        posX = limelight.getX();
//        posY = limelight.getY();
        //Drive:
        x = gamepad1.left_stick_x;
        y = gamepad1.left_stick_y;
        rx = gamepad1.right_stick_x;
        drive.driveSmooth(x, y, rx);
        //shooter.automatic();
        //Turret:
        if(gamepad2.dpad_left){
            turret.moveLeft();
        }else if(gamepad2.dpad_right){
            turret.moveRight();
        }
        //Shooter:
        if(gamepad2.cross){
            shooter.override = true;
            shooter.low();
        }else if(gamepad2.square){
            shooter.override = true;
            shooter.mid();
        }else if(gamepad2.triangle){
            shooter.override = true;
            shooter.high();
        }else if(gamepad2.circleWasPressed()){
            shooter.override = false;
//            shooter.automatic();
        }else{
            shooter.stop();
        }
        //Intake:
        if(gamepad1.right_bumper){
            intake.run();
        }else if(gamepad1.left_bumper){
            intake.reverse();
        }else{
            intake.stop();
        }
        if(gamepad2.rightBumperWasPressed()){
            t.resetTimer();
            if(t.getElapsedTimeSeconds() < 0.2){
                transfer.run();
            }else if(t.getElapsedTimeSeconds() > 0.2){
                transfer.run();
                gate.open();
            }
        }
        if(gamepad2.rightBumperWasReleased()){
            transfer.stop();
            gate.close();
        }
        if(gamepad2.left_bumper){
            gate.close();
            transfer.reverse();
        }
        telemetry.addData("LimelightXPos: ", posX);
        telemetry.addData("LimelightYPos: ", posY);
    }
}
