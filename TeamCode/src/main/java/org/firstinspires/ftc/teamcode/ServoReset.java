package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.hardware.AbsoluteAnalogEncoder;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@Config
@Configurable
@TeleOp(name = "ServoResetPos", group = "Test")
public class ServoReset extends OpMode {
    private ServoEx servo;
    private double pos = 0.1;
    private static AbsoluteAnalogEncoder turretEncoder;
    private Intake intake;
    private static CRServoEx turret;
    private static PIDFCoefficients coefficients;
    private static double P = 0.00001;
    private static double I = 0.5;
    private static double D = 0.5;
    private static double F = 0.016;
    double sp = 180;
    @Override
    public void init() {
        //absolute value of the pid output or input, ////////**preferably input
        servo = new ServoEx(hardwareMap, "hood");
        servo.set(pos);
        intake = new Intake(hardwareMap);
        turretEncoder = new AbsoluteAnalogEncoder(hardwareMap, "turretEncoder", 3.3, AngleUnit.DEGREES);
        turret = new CRServoEx(hardwareMap, "turret", turretEncoder, CRServoEx.RunMode.OptimizedPositionalControl);
        turret.setCachingTolerance(0.0002);
        turretEncoder.setReversed(true);
//        turret.setPIDF(coefficients);
    }//if lastValue was >3.2 and current value < 0.2 counter++


    @Override
    public void loop() {
        coefficients = new PIDFCoefficients(P, I, D, F);
        turret.setPIDF(coefficients);

        if (gamepad1.dpadLeftWasPressed()) {
            pos += 0.02;
        }
        if (gamepad1.dpadRightWasPressed()) {
            pos -= 0.02;
        }
        if (gamepad1.dpadDownWasPressed()) {
            pos -= 0.01;
        }
        if (gamepad1.dpadUpWasPressed()) {
            pos += 0.01;
        }
        if(gamepad1.dpad_left){
            sp-= 0.1;
        } else if (gamepad1.dpad_right) {
            sp+= 0.1;
        }
        if (gamepad1.right_bumper) {
            intake.run();
        }else{
            intake.stop();
        }
        if(gamepad1.crossWasPressed()){
            sp = 100;
        }
        if(gamepad1.leftBumperWasPressed()){
            sp = 200;
        }
        turret.set(sp);
        servo.set(pos);
        telemetry.addData("GetCurrentPosition", turretEncoder.getCurrentPosition());
        telemetry.addData("SP", sp);
        telemetry.addData("DeviceType", turretEncoder.getDeviceType());
        telemetry.addData("Encoder voltage", turretEncoder.getVoltage());

        telemetry.addData("Servo: " , " ");
        telemetry.addData("ServoType" , turret.getServo());
        telemetry.addData("ServoPos", servo.getRawPosition());
        telemetry.addData("Pos", pos);
    }
}