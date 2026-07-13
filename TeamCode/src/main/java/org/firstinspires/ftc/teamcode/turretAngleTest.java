package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.hardware.motors.CRServoEx;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;


@TeleOp
@Config
@Configurable
public class turretAngleTest extends LinearOpMode {
    private Turret turret;
    private Follower follower;

    @Override
    public void runOpMode() throws InterruptedException {

        turret = new Turret(hardwareMap);
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(72, 72));
        follower.setHeading(90);

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            turret.run();
            telemetry.addData("UnwrappedAngle", turret.getUnwrappedServoDegrees());
            telemetry.update();
        }
    }
}
