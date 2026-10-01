package org.firstinspires.ftc.teamcode.TeleOops;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.seattlesolvers.solverslib.gamepad.GamepadEx;
import com.seattlesolvers.solverslib.gamepad.GamepadKeys;
import com.seattlesolvers.solverslib.gamepad.ToggleButtonReader;

import org.firstinspires.ftc.teamcode.Subsystems.Drive;
import org.firstinspires.ftc.teamcode.Subsystems.Gate;
import org.firstinspires.ftc.teamcode.Subsystems.Hood;
import org.firstinspires.ftc.teamcode.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.Subsystems.Shooter;
import org.firstinspires.ftc.teamcode.Subsystems.Transfer;
import org.firstinspires.ftc.teamcode.Subsystems.Turret;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "main")
public class TeleOp extends OpMode {
    private Turret turret;
    private Drive drive;
    private Shooter shooter;
    private Intake intake;
    private Transfer transfer;
    private Gate gate;
    private Limelight limelight;
    private Hood hood;
    private double pos;
    private Timer t;
    double posX, posY;
    private Follower follower;
    private GamepadEx gamepadA, gamepadB;
    private ToggleButtonReader shooterHigh, shooterMid, shooterLow;
    @Override
    public void init() {
        gamepadA = new GamepadEx(gamepad1);
        gamepadB = new GamepadEx(gamepad2);
        shooterHigh = new ToggleButtonReader(gamepadB, GamepadKeys.Button.TRIANGLE);
        shooterMid = new ToggleButtonReader(gamepadB, GamepadKeys.Button.SQUARE);
        shooterLow = new ToggleButtonReader(gamepadB, GamepadKeys.Button.CROSS);
        turret = new Turret(hardwareMap);
        drive = new Drive(hardwareMap);
        shooter = new Shooter(hardwareMap);
        intake = new Intake(hardwareMap);
        gate = new Gate(hardwareMap);
        transfer = new Transfer(hardwareMap);
        limelight = new Limelight(hardwareMap);
        hood = new Hood(hardwareMap);
        t = new Timer();
        transfer.stop();
        shooter.stop();
        intake.stop();
        follower = Constants.createFollower(hardwareMap);

//        follower.setStartingPose(new Pose(72, 72, Math.toRadians(90)));
        turret.setAlliance(Turret.Alliance.RED);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());
    }

    @Override
    public void loop() {
        follower.update();
        gamepadA.readButtons();
        gamepadB.readButtons();
        shooterMid.readValue();
        shooterHigh.readValue();
        shooterLow.readValue();

//        limelight.see(follower.getHeading());
//        posX = limelight.getX();
//        posY = limelight.getY();
        //Drive:
        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double rx = gamepad1.right_stick_x;
        shooter.setFromDistance(turret.getDistanceFromBasket());
        drive.driveSmooth(x, y, rx);
        //shooter.automatic();
        //Turret:
        if(gamepad1.dpadDownWasPressed()){
        Turret.TEST = !Turret.TEST;
        }
        if(gamepad1.dpad_left){
            turret.gamepadMoveFreelyLeft();
        }else if(gamepad1.dpad_right){
            turret.gamepadMoveFreelyRight();
        }else{
            turret.gamepadStop();
        }
        turret.aimAtBasket(follower);
        if(gamepad2.dpad_left){
            turret.gamepadOffsetLeft();
        }else if(gamepad2.dpad_right){
            turret.gamepadOffsetRight();
        }

        //Shooter:
//        if(shooterLow.getState()){
//            shooter.low();
//        }else if(shooterMid.getState()){
////            shooter.override = true;
//            shooter.mid();
//        }else if(shooterHigh.getState()){
////            shooter.override = true;
//            shooter.high();
////        }else if(gamepad2.circleWasPressed()){
////            shooter.override = false;
////            shooter.automatic();
//        }else{
//            shooter.stop();
//        }
        if(gamepad2.right_trigger_pressed){
            shooter.gamepadIncreaseRPM();
        }else if(gamepad2.left_trigger_pressed){
            shooter.gamepadDecreaseRPM();
        }else if(gamepad2.squareWasPressed()){
            shooter.resetGamepadOffset();
        }
        if(gamepad2.leftBumperWasPressed()){
            turret.setAlliance(Turret.Alliance.BLUE);
        }
        if(gamepad2.rightBumperWasPressed()){
            turret.setAlliance(Turret.Alliance.RED);
        }
        if(gamepad1.crossWasPressed()){
            follower.setPose(new Pose(127, 63, 0));
            turret.resetGamepadOffset();
        }else if(gamepad1.squareWasPressed()){
            follower.setPose(new Pose(15, 63, Math.toRadians(180)));
            turret.resetGamepadOffset();
            //Blue human
        }else if(gamepad1.circleWasPressed()){
            follower.setPose(new Pose(8,8, Math.toRadians(180)));
            turret.resetGamepadOffset();
            //Red human
        }else if(gamepad1.triangleWasPressed()){
            follower.setPose(new Pose(135,8, Math.toRadians(0)));
            turret.resetGamepadOffset();
        }
        if(gamepad2.dpadDownWasPressed()){
            turret.resetGamepadOffset();
        }
        //Intake:
        if(gamepad1.right_bumper){
            intake.run();
            transfer.stall();
        }else if(gamepad1.left_bumper){
            intake.reverse();
            transfer.stall();
        }else{
            intake.stop();
            transfer.stop();
        }
        if(gamepad1.rightTriggerWasPressed()){
            t.resetTimer();
        }
        if (gamepad1.right_trigger_pressed && t.getElapsedTimeSeconds() < 0.06) {
                gate.open();
        }else if(gamepad1.right_trigger_pressed && t.getElapsedTimeSeconds() > 0.06){
            transfer.run();
            intake.run();
        }
        if(gamepad1.rightTriggerWasReleased()){
            transfer.stop();
            gate.close();
        }
        if(gamepad1.left_trigger_pressed){
            gate.close();
            transfer.reverse();
        }
        turret.run();
        hood.run(turret, shooter);

        telemetry.addData("RobotPose", follower.getPose());
        telemetry.addData("Unwrapped servo degrees ", turret.getUnwrappedServoDegrees());
        telemetry.addData("targetServoDegrees", turret.getTargetServoDegrees());
        telemetry.addData("ErrorDegrees", turret.getErrorDegrees());
        telemetry.addData("TargetPhysicalAngle", turret.getTargetPhysicalAngle());
        telemetry.addData("PhysicalAngle", turret.getCurrentPhysicalAngle());
        telemetry.addData("heading", follower.getHeading());
        telemetry.addData("LimelightYPos: ", posY);
        telemetry.addData("ID: ", limelight.getID());
        telemetry.addData("RPM", shooter.getRPM());
        telemetry.addData("TargetRPM", shooter.getTarget());
        telemetry.update();


    }
}
