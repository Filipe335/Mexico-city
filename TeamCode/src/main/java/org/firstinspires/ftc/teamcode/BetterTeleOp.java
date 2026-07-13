package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;

@Configurable
@Config
@TeleOp(name = "AutomatedTeleOp")
public class BetterTeleOp extends OpMode {
    private Drive drive;
    private Intake intake;
    private Transfer transfer;
    private Gate gate;
    private Turret turret;
    private Shooter shooter;
    private Limelight limelight;
    private GamepadEx gamepadA, gamepadB;
    private boolean on = false;
    private Timer t;
    private double x, y, rx;
    @Override
    public void init() {
        gamepadA = new GamepadEx(gamepad1);
        gamepadB = new GamepadEx(gamepad2);
        drive = new Drive(hardwareMap);
        intake = new Intake(hardwareMap);
        transfer = new Transfer(hardwareMap);
        gate = new Gate(hardwareMap);
        turret = new Turret(hardwareMap);
        shooter = new Shooter(hardwareMap);
        limelight = new Limelight(hardwareMap);
    }

    @Override
    public void loop() {
    gamepadA.readButtons();
    gamepadA.readButtons();

    x = gamepad1.left_stick_x;
    y = gamepad1.left_stick_y;
    rx = gamepad1.right_stick_x;
    drive.driveSmooth(x, y, rx);

        //The on = !on acts as a toggle logic
        if(gamepad1.leftTriggerWasPressed()){
            on = !on;
            shooter.mid();//change to automatic when lut is done
        }
        if(!on){
            shooter.stop();
        }
        //Intake
        if(gamepad1.right_bumper){
            intake.run();
        }else{
            intake.stop();
        }
        //Shoot logic. When the trigger is pressed, the timer is reset.
        //While the right trigger is pressed, execute the automation
        if(gamepad1.rightTriggerWasPressed()){
            t.resetTimer();
        }
        if(gamepad1.right_trigger_pressed && t.getElapsedTimeSeconds() < 0.1){
            gate.open();
        }else if(gamepad1.right_trigger_pressed && t.getElapsedTimeSeconds() > 0.1){
            gate.open();
            intake.run();
            transfer.run();
        }
        if(gamepad1.rightTriggerWasReleased()){
            gate.close();
            intake.stop();
            transfer.stop();
        }



    }
}
