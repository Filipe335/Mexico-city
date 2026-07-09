package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.controller.PIDFController;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;
@Config
@Configurable
public class Shooter extends SubsystemBase {
    private MotorEx shooter;
    private static PIDFController pid;
    private static double P;
    private static double I;
    private static double D;
    private static double F;
    public boolean override;

    public Shooter(HardwareMap hardwareMap){
        shooter = new MotorEx(hardwareMap, "shooter");
        shooter.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        shooter.setRunMode(Motor.RunMode.RawPower);
        pid = new PIDFController(P, I, D, F);

    }
    public void automatic(){
        if(!override){
            shooter.set(pid.calculate(shooter.getVelocity()));
        }
    }
    public void low(){
        shooter.set(pid.calculate(800));
    }
    public void mid(){
        shooter.set(pid.calculate(1300));
    }
    public void high(){
        shooter.set(pid.calculate(1800));
    }
    public void stop(){
        shooter.set(0);
    }
}
