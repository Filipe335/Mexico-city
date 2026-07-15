package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.seattlesolvers.solverslib.command.SubsystemBase;
import com.seattlesolvers.solverslib.hardware.servos.ServoEx;
import com.seattlesolvers.solverslib.util.MathUtils;

public class Hood extends SubsystemBase {
    private ServoEx hood;
    double correction;
    public Hood(HardwareMap hardwareMap){
        hood = new ServoEx(hardwareMap, "hood");
        hood.setCachingTolerance(0.00005);
    }
    public void run(Turret turret, Shooter shooter){
        double error = shooter.getRPMError();
        if(error <= 0){
            correction = error / 1800; // increase this value for stronger correction
        }
        double p = 0.0063*(turret.getDistanceFromBasket() - 33);
        if(p > 0 && p < 0.8){
            double result = p - correction;
            double pos = MathUtils.clamp(result, 0, 0.8);
            hood.set(pos);
        }else{
            hood.set(0);
        }

    }
}
