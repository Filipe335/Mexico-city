package org.firstinspires.ftc.teamcode;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.util.Timing;

public class Turret extends SubsystemBase {
    private MotorEx turret;
    double sp = 0;
    private Timer t;
    double elapsed = 0;
    double motorSpeed = 0;
    double output = 0;
    public int counter = 0;
    public Turret(HardwareMap hardwareMap){
        turret = new MotorEx(hardwareMap, "turret");
        turret.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        turret.setInverted(false);
        turret.setRunMode(Motor.RunMode.RawPower);
        t = new Timer();
    }
//    public void moveLeft(){
//        elapsed = t.getElapsedTimeSeconds();
//        output = 0.8 * counter/elapsed;
//    }
    public void moveLeft(){
        sp += 0.005;
        turret.set(sp);
    }
    public void moveRight(){
        sp -= 0.005;
        turret.set(sp);
    }
    public void increaser(){
        counter += 1;
    }
}
