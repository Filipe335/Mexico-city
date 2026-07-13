package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
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
    private static PIDFController pid;
    private static double P = 0.07;
    private static double I = 0.00005;
    private static double D = 0.00005;
    private static double F = 0.07;
    public boolean override;
    private ServoEx hood;

    public Shooter(HardwareMap hardwareMap){
        shooter = new MotorEx(hardwareMap, "shooter");
        shooter2 = new MotorEx(hardwareMap, "shooter2");
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter2.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter.setRunMode(Motor.RunMode.RawPower);
        shooter2.setRunMode(Motor.RunMode.RawPower);
        shooter.setInverted(false);
        pid = new PIDFController(P, I, D, F);
        InterpLUT lut = new InterpLUT();
        hood = new ServoEx(hardwareMap, "hood");
        hood.setCachingTolerance(0.00005);
//        lut.add();

    }
    public void automatic(){
        if(!override){
            shooter.set(pid.calculate(shooter.getVelocity()));
        }
    }
    public void low(){
        shooter.set(0.7);
        shooter2.set(0.7);
    }
    public void mid(){
        shooter.set(pid.calculate(ticksToRPM(shooter2.getVelocity()), 4500));
        shooter2.set(pid.calculate(ticksToRPM(shooter2.getVelocity()), 4500));
    }
    public void high(){
        shooter.set(pid.calculate(ticksToRPM(shooter2.getVelocity()), 5500));
        shooter2.set(pid.calculate(ticksToRPM(shooter2.getVelocity()), 5500));
    }
    public void stop(){
        shooter.set(0);
        shooter2.set(0);
    }
    public int ticksToRPM(double ticks){
        return (int)ticks * 31/14;
    }
    public int getRPM(){
        return ticksToRPM(shooter2.getVelocity());
    }

}
