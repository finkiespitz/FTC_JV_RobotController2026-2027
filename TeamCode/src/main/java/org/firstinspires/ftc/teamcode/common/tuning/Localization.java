package org.firstinspires.ftc.teamcode.common.tuning;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

@Config
public class Localization {
    public GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry;

    public static double START_X = 0.0;
    public static double START_Y = 0.0;
    public static double START_HEADING = 0.0;

    public Localization(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        pinpoint.setOffsets(-100.0, 100.0, DistanceUnit.INCH);
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        Pose2D startPose = new Pose2D(DistanceUnit.INCH, START_X, START_Y, AngleUnit.DEGREES, START_HEADING);
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

    public double getX() {
        return pinpoint.getPosition().getX(DistanceUnit.INCH);
    }

    public double getY() {
        return pinpoint.getPosition().getY(DistanceUnit.INCH);
    }

    public double getHeading() {
        return pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }
}
