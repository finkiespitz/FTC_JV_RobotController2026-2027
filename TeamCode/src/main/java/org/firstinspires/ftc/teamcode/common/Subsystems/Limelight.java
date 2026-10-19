package org.firstinspires.ftc.teamcode.common.Subsystems;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes; // NEW: Needed to scan AprilTag blocks
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import java.util.List;

public class Limelight {
    public Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");

        limelight.pipelineSwitch(0); // Make sure pipeline 0 is set to AprilTags in the Limelight UI
        limelight.start(); // Starts polling data
        limelight.pipelineSwitch(0);
    }


    public double getPose() {
        LLResult result = limelight.getLatestResult();

        if (result != null && result.isValid()) {


            List<LLResultTypes.FiducialResult> AprilTags = result.getFiducialResults();


            if (!AprilTags.isEmpty()) {
                //Grab the first AprilTag detected in the frame
                LLResultTypes.FiducialResult targetTag = AprilTags.get(0);

                //Get the tag's position relative to the camera lens
                Pose3D cameraPose = targetTag.getCameraPoseTargetSpace();

                if (cameraPose != null) {
                    //Extract raw positioning coordinates (SDK maps positions using standard primitive doubles .x, .y, .z)
                    double x = cameraPose.getPosition().x;
                    double y = cameraPose.getPosition().y;
                    double z = cameraPose.getPosition().z;

                    //3D Pythagorean theorem to find the true straight-line distance
                    double distanceFromHive = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));

                    return distanceFromHive;
                }
            }
        }

        // Return -1.0 if nothing is found
        return 0;
    }
}