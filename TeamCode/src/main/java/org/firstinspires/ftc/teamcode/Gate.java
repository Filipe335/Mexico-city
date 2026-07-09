package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;

public class Gate extends SubsystemBase {
    private ServoEx gate;
    private double DEFAULT, OPEN, CLOSE;
    public Gate(HardwareMap hardwareMap){
        gate = new ServoEx(hardwareMap, "gate");
        gate.setCachingTolerance(0.00001);
        gate.set(CLOSE);
    }
    public void open(){
        gate.set(OPEN);
    }
    public void close(){
        gate.set(CLOSE);
    }
}
