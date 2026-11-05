package org.firstinspires.ftc.teamcode.common.tuning;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.common.subsystems.Limelight;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.List;

@Config
public class Localization {
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;

    public Localization(HardwareMap hardwareMap, Telemetry telemetry, boolean redTeam) {
        this.telemetry = telemetry;

        double startX;
        double startY;
        double startHeading = 0.0;

        if (redTeam) {
            startX = 15.7594;
            startY = 119.9135;
        } else {
            startX = 125.7406;
            startY = 21.5865;
        }

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-0.8444454914002907, -1.4199307959849443, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        Pose2D startPose = new Pose2D(DistanceUnit.INCH, startX, startY, AngleUnit.DEGREES, startHeading);
        pinpoint.setPosition(startPose);
    }

    public void update() {
        pinpoint.update();

        Pose2D pose = pinpoint.getPosition();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double headingDeg = pose.getHeading(AngleUnit.DEGREES);

        telemetry.addData("X Position (in)", x);
        telemetry.addData("Y Position (in)", y);
        telemetry.addData("Heading (deg)", headingDeg);
        telemetry.update();

        TelemetryPacket packet = new TelemetryPacket();
        packet.put("Robot X", x);
        packet.put("Robot Y", y);
        packet.put("Robot Heading (deg)", headingDeg);

        packet.fieldOverlay()
                .setStroke("blue")
                .setStrokeWidth(1)
                .strokeCircle(x, y, 9)
                .strokeLine(x, y, x + 9 * Math.cos(Math.toRadians(headingDeg)), y + 9 * Math.sin(Math.toRadians(headingDeg)));

        FtcDashboard.getInstance().sendTelemetryPacket(packet);
    }

    public void correctHeadingWithVision(Limelight limelight, boolean redTeam) {

       LLResult result = limelight.limelight.getLatestResult();
       if (result != null && result.getBotpose() != null) {
           List<LLResultTypes.FiducialResult> AprilTags = result.getFiducialResults();

           LLResultTypes.FiducialResult targetTag = AprilTags.get(0);
           int tagId = targetTag.getFiducialId();

           double visionHeadingRad;
           if(tagId > 33 && tagId < 42){
               visionHeadingRad = result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS) + 90;

               pinpoint.setHeading(visionHeadingRad, AngleUnit.RADIANS);
           }
           else {
               visionHeadingRad = result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS) - 90;

               pinpoint.setHeading(visionHeadingRad, AngleUnit.RADIANS);
           }
       }
    }

    public double getX() { 
        return pinpoint.getPosition().getX(DistanceUnit.INCH); 
    }
    
    public double getY() { 
        return pinpoint.getPosition().getY(DistanceUnit.INCH); 
    }
    
    public double getHeadingRad() { 
        return pinpoint.getPosition().getHeading(AngleUnit.RADIANS); 
    }
    
    public void resetHeading() {
        pinpoint.recalibrateIMU();
    }
}
