package org.firstinspires.ftc.teamcode.HelperClasses;

import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;

public class AutoHelperClass {
    private Shooter shooter;
    private Turret turret;
    private Gate gate;
    private Transfer transfer;
    private Intake intake;
    private Timer t;
    private boolean active = false;
    public AutoHelperClass(Shooter shooter, Turret turret, Gate gate, Transfer transfer, Intake intake) {
        this.shooter = shooter;
        this.turret = turret;
        this.gate = gate;
        this.transfer = transfer;
        this.intake = intake;
        t = new Timer();
    }
    public void shoot(){
        if(!active){
            gate.open();
            active = true;
        }
        transfer.run();
    }

}
