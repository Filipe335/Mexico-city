package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight {
    private Limelight3A limelight;
    private LLResult result;
    private LLResult latestResult;
    private double tx, ty, ta;
    private double x, y;
    private double distance;
    private double METERS_TO_INCH = 1 / 256.4;
    int id = 0;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100);
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public void see(double heading) {
        limelight.updateRobotOrientation(heading);
        result = limelight.getLatestResult();
        x = result.getBotpose_MT2().getPosition().x * METERS_TO_INCH;
        y = result.getBotpose_MT2().getPosition().y * METERS_TO_INCH;
        List<LLResultTypes.FiducialResult> fiducials = result.getFiducialResults();
        for (LLResultTypes.FiducialResult fiducial : fiducials) {
            id = fiducial.getFiducialId(); // The ID number of the fiducial
//            double x = detection.getTargetXDegrees(); // Where it is (left-right)
//            double y = detection.getTargetYDegrees(); // Where it is (up-down)
            distance = fiducial.getRobotPoseTargetSpace().getPosition().y;
        }
    }

    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }
    public int getID(){
        return id;
    }
    public double getDistance() {
        return distance;
    }
}