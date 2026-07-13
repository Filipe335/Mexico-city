package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name = "ShooterTest")
public class ShooterTest extends OpMode {
    private MotorEx shooter;
    @Override
    public void init() {
    shooter = new MotorEx(hardwareMap, "shooter");
    shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
    }

    @Override
    public void loop() {
        shooter.set(1);
        telemetry.addData("ticks", shooter.getVelocity());
    }
}
