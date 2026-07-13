package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.InterpLUT;

@Config
@Configurable
public class Shooter extends SubsystemBase {
    private MotorEx shooter;
    private MotorEx shooter2;
    public static PIDFController pid;
    public static double P = 0.004;
    public static double I = 0.0001;
    public static double D = 0.0001;
    public static double F = 0.000195;
    public boolean override;
    private ServoEx hood;

    public Shooter(HardwareMap hardwareMap){
        shooter = new MotorEx(hardwareMap, "shooter");
        shooter2 = new MotorEx(hardwareMap, "shooter2");
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter2.setRunMode(Motor.RunMode.RawPower);
        shooter2.setInverted(true);
        pid = new PIDFController(P, I, D, F);
        InterpLUT lut = new InterpLUT();
        hood = new ServoEx(hardwareMap, "hood");
        hood.setCachingTolerance(0.00005);
//        lut.add();

    }
    public void automatic(){
        if(!override){
            shooter.set(pid.calculate(shooter2.getVelocity()));
        }
    }
    public void low(){
        pid.setCoefficients(new PIDFCoefficients(P, I, D, F));
        shooter.set(pid.calculate(ticksToRPM(shooter2.getVelocity()), 1000));
        shooter2.set(pid.calculate(ticksToRPM(shooter2.getVelocity()), 1000));
    }
    public void mid(){
        pid.setCoefficients(new PIDFCoefficients(P, I, D, F));
        shooter.set(pid.calculate(ticksToRPM(shooter2.getVelocity()), 2500));
        shooter2.set(pid.calculate(ticksToRPM(shooter2.getVelocity()), 2500));
    }
    public void high(){
        pid.setCoefficients(new PIDFCoefficients(P, I, D, F));
        shooter.set(pid.calculate(ticksToRPM(shooter2.getVelocity()), 4000));
        shooter2.set(pid.calculate(ticksToRPM(shooter2.getVelocity()), 4000));
    }
    public void stop(){
        shooter.set(0);
        shooter2.set(0);
    }
    public int ticksToRPM(double ticks){
        return -(int)ticks * 31/14;
    }
    public int getRPM(){
        return ticksToRPM(shooter2.getVelocity());
    }
    public double getTarget(){
        return pid.getSetPoint();
    }

}
