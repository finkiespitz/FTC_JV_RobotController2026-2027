package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes; // NEW: Needed to scan AprilTag blocks
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight {
    public Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0); // set pipeline 0 to AprilTags in the Limelight UI
        limelight.start(); // Starts polling data
    }


    public double getPose(boolean redTeam) {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return 0;
        }

        List<LLResultTypes.FiducialResult> AprilTags = result.getFiducialResults();
        if (AprilTags == null || AprilTags.isEmpty()) {
            return 0;
        }

        LLResultTypes.FiducialResult targetTag = AprilTags.get(0);
        int tagId = targetTag.getFiducialId();

        // Validate Tag ID based on Alliance Team
        if (redTeam) {
            if (tagId < 30 || tagId > 37) return 0;
        } else {
            if (tagId < 38 || tagId > 45) return 0;
        }

        // Calculate straight-line distance
        Pose3D cameraPose = targetTag.getRobotPoseTargetSpace();
        if (cameraPose != null) {
            double x = cameraPose.getPosition().x;
            double y = cameraPose.getPosition().y;
            double z = cameraPose.getPosition().z;

            return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));
        }

        return 0;
    }
}