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
    private Limelight localLimelight;
    private final boolean redTeam;
    private final GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;
    private double x = 0.0;
    private double y = 0.0;
    private final double RED_HIVE_AUDIENCE_SIDE_X = 57.7;
    private final double RED_HIVE_AUDIENCE_SIDE_Y = 86.5;
    private final double RED_HIVE_BACK_SIDE_X = 57.7;
    private final double RED_HIVE_BACK_SIDE_Y = 55.0;
    private final double BLUE_HIVE_AUDIENCE_SIDE_Y = 86.7;
    private final double BLUE_HIVE_AUDIENCE_SIDE_X = 83.8;
    private final double BLUE_HIVE_BACK_SIDE_Y = 55.0;
    private final double BLUE_HIVE_BACK_SIDE_X = 83.8;
    private boolean isAudienceSide;

    public Localization(HardwareMap hardwareMap, Telemetry telemetry, boolean teleOpRedTeam, Limelight Limelight) {
        localLimelight = Limelight;
        this.telemetry = telemetry;
        redTeam = teleOpRedTeam;
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
        correctHeadingWithVision(localLimelight, redTeam);

        Pose2D pose = pinpoint.getPosition();
        x = pose.getX(DistanceUnit.INCH);
        y = pose.getY(DistanceUnit.INCH);
        double headingDeg = pose.getHeading(AngleUnit.DEGREES);

        telemetry.addData("X Position (in)", x);
        telemetry.addData("Y Position (in)", y);
        telemetry.addData("Heading (deg)", headingDeg);
    }

    public void correctHeadingWithVision(Limelight limelight, boolean redTeam) {

        LLResult result = limelight.limelight.getLatestResult();

        if (result == null || !result.isValid()) {
            return;
        }

        if (result.getBotpose() == null) {
            return;
        }

        List<LLResultTypes.FiducialResult> aprilTags =
                result.getFiducialResults();

        // Make sure at least one AprilTag was detected
        if (aprilTags == null || aprilTags.isEmpty()) {
            return;
        }

        LLResultTypes.FiducialResult targetTag = aprilTags.get(0);

        int tagId = targetTag.getFiducialId();

        double visionHeadingRad;

        if (tagId > 33 && tagId < 42) {
            visionHeadingRad =
                    result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)
                            + Math.toRadians(90);
        } else {
            visionHeadingRad =
                    result.getBotpose().getOrientation().getYaw(AngleUnit.RADIANS)
                            - Math.toRadians(90);
        }

        pinpoint.setHeading(visionHeadingRad, AngleUnit.RADIANS);
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

    public double[] getHiveTarget(boolean redTeam, double localY) {
        isAudienceSide = localY < 70.75;

        if (redTeam) {
            double targetX = isAudienceSide ? RED_HIVE_AUDIENCE_SIDE_X : RED_HIVE_BACK_SIDE_X;
            double targetY = isAudienceSide ? RED_HIVE_AUDIENCE_SIDE_Y : RED_HIVE_BACK_SIDE_Y;
            return new double[]{targetX, targetY};
        } else {
            double targetX = isAudienceSide ? BLUE_HIVE_AUDIENCE_SIDE_X : BLUE_HIVE_BACK_SIDE_X;
            double targetY = isAudienceSide ? BLUE_HIVE_AUDIENCE_SIDE_Y : BLUE_HIVE_BACK_SIDE_Y;
            return new double[]{targetX, targetY};
        }
    }
    public double getDeltaXToHive(boolean redTeam, double localX, double localY){
        double[] hiveXY = getHiveTarget(redTeam, localY);
        return hiveXY[0] - localX;
    }
    public double getDeltaYToHive(boolean redTeam, double localY){
        double[] hiveXY = getHiveTarget(redTeam, localY);
        return hiveXY[1] - localY;
    }
    public double getRobotHeadingOffsetToHive (boolean redTeam, double localX, double localY){
        //θ 1
        return Math.toDegrees(Math.atan2(getDeltaXToHive(redTeam, localX, localY), getDeltaYToHive(redTeam, localY)));
    }
    public double getDistanceToHive(boolean redTeam, double localX, double localY) {
        return Math.hypot(getDeltaXToHive(redTeam, localX, localY), getDeltaYToHive(redTeam, localY));
    }

}

