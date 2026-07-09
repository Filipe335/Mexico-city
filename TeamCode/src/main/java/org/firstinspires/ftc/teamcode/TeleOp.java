package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "main")
public class TeleOp extends OpMode {
    private Turret turret;
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;
    private Gate gate;
    private Timer t;
    double x, y, rx;
    @Override
    public void init() {
        turret = new Turret(hardwareMap);
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        transfer = new Transfer(hardwareMap);
        transfer.stop();
        shooter.stop();
        intake.stop();
    }

    @Override
    public void loop() {
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
        if(gamepad2.crossWasPressed()){
            shooter.override = true;
            shooter.low();
        }else if(gamepad2.squareWasPressed()){
            shooter.override = true;
            shooter.mid();
        }else if(gamepad2.triangleWasPressed()){
            shooter.override = true;
            shooter.high();
        }else if(gamepad2.circleWasPressed()){
            shooter.override = false;
            shooter.automatic();
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
    }
    @Override
    public void stop(){
        turret.counter = 0;
    }
}
