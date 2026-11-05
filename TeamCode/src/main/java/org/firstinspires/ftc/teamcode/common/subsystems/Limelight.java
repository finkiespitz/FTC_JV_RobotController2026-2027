package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

public class Limelight {
    public Limelight3A limelight;

    public Limelight(HardwareMap hardwareMap) {
        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.pipelineSwitch(0);
        limelight.start();
    }

    public String teamFilter(boolean redTeam) {
        LLResult result = limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return "notValid";
        }

        List<LLResultTypes.FiducialResult> AprilTags = result.getFiducialResults();
        if (AprilTags == null || AprilTags.isEmpty()) {
            return "notValid";
        }

        LLResultTypes.FiducialResult targetTag = AprilTags.get(0);
        int tagId = targetTag.getFiducialId();

        if (redTeam) {
            if (tagId < 30 || tagId > 37) return "notValid";
        } else {
            if (tagId < 38 || tagId > 45) return "notValid";
        }
        return "valid";
    }

    public double getPose(boolean redTeam) {
        LLResult result = limelight.getLatestResult();	
        String validity = teamFilter(redTeam);
	
        if (validity.equals("valid") || result == null) {
            List<LLResultTypes.FiducialResult> AprilTags = null;
            if (result != null) {
                AprilTags = result.getFiducialResults();
            }
            if (AprilTags == null || AprilTags.isEmpty()) return 1000;
            
            LLResultTypes.FiducialResult targetTag = AprilTags.get(0);
            Pose3D cameraPose = targetTag.getRobotPoseTargetSpace();
            if (cameraPose != null) {
                double x = cameraPose.getPosition().x;
                double y = cameraPose.getPosition().y;
                double z = cameraPose.getPosition().z;

                return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) + Math.pow(z, 2));}
	}
       	return 1000;
    }

    public double getTx(boolean redTeam) {
        LLResult result = limelight.getLatestResult();
        String validity = teamFilter(redTeam);

        if (validity.equals("valid") || result == null) {
            if (result != null) {
                return result.getTx();
            }

        }
        return 0;
    }

    public double getTy(boolean redTeam) {
        LLResult result = limelight.getLatestResult();
        String validity = teamFilter(redTeam);

        if (validity.equals("valid") || result == null) {
            if (result != null) {
                return result.getTy();
            }
        }
        return 0;
    }

    public double getTa(boolean redTeam) {
        LLResult result = limelight.getLatestResult();
        String validity = teamFilter(redTeam);

        if (validity.equals("valid") || result == null) {
            if (result != null) {
                return result.getTa();
            }
        }
        return 0;
    }

    public double getTargetYaw(boolean redTeam) {
        LLResult result = limelight.getLatestResult();
        String validity = teamFilter(redTeam);

        if (validity.equals("valid")) {
           return result.getBotpose().getOrientation().getYaw(AngleUnit.DEGREES);
        }
        return 0;
    }
}