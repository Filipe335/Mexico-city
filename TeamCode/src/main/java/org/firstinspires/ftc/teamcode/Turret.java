package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.Timing;
//IMPORTANT import this then: MathUtils.clamp
import com.seattlesolvers.solverslib.util.MathUtils;

//IMPORTANT import this then: MathUtils.clamp


public class Turret extends SubsystemBase {
    private ServoEx turret;
    double sp = 0;
    double targetAngle;

    private Timer t;
    private Pose RED_BASKET;
    private Pose BLUE_BASKET;
    double elapsed = 0;
    double motorSpeed = 0;
    double output = 0;
    public int counter = 0;
    public Turret(HardwareMap hardwareMap){
        turret = new ServoEx(hardwareMap, "turret");
        t = new Timer();
        RED_BASKET = new Pose(138, 138);
        BLUE_BASKET = new Pose(2, 138);
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

//    public double calculate(Follower follower){
//        double XDifference = BLUE_BASKET.getX() - follower.getPose().getX();
//        double YDifference = BLUE_BASKET.getY() - follower.getPose().getY();
//        double target = Math.atan(YDifference/XDifference);
//        double result = target - follower.getHeading();
////        double z = follower.getHeading() -
//        return 2;//change
//    }
    public void increaser(){
        counter += 1;
    }
    public void update(double target){

    }
}
