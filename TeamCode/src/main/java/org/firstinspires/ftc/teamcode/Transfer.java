package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

public class Transfer extends SubsystemBase {
    private MotorEx transfer;
    public Transfer(HardwareMap hardwareMap){
        transfer = new MotorEx(hardwareMap, "transfer");
        transfer.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        transfer.setRunMode(Motor.RunMode.RawPower);
        transfer.setInverted(false);
    }
    public void run(){
        transfer.set(1);
    }
    public void reverse(){
        transfer.set(-1);
    }
    public void stop(){
        transfer.set(0);
    }
    public void stall(){
        transfer.set(-0.4);
    }
}
