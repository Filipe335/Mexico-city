package org.firstinspires.ftc.teamcode.Subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
@Configurable
@Config
public class Drive {
    private MotorEx leftFront, leftBack, rightFront, rightBack;
    private static double P = 0.8;
    private static double I = 0.0002;
    private static double D = 0.0002;
    private static double F = 0.9;
    private PIDFController pid;
    double lf, lb, rf, rb;
    public boolean slow = false;

    public Drive(HardwareMap hardwareMap){
        leftFront = new MotorEx(hardwareMap, "leftFront");
        leftBack = new MotorEx(hardwareMap, "leftBack");
        rightFront = new MotorEx(hardwareMap, "rightFront");
        rightBack = new MotorEx(hardwareMap, "rightBack");
        leftFront.setInverted(true);
        leftBack.setInverted(true);
        pid = new PIDFController(P, I, D, F);
        leftFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightFront.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
    }
    public void driveSmooth(double x, double y, double rx){
        lf= y + x + rx;
        rf= y - x - rx;
        lb= y - x + rx;
        rb= y + x - rx;
        leftFront.set(lf);
        leftBack.set(lb);
        rightFront.set(rf);
        rightBack.set(rb);
    }
}
