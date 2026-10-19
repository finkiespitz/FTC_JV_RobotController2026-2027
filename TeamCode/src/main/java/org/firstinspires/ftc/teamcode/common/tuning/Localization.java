package org.firstinspires.ftc.teamcode.common.tuning;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Localization {
    public GoBildaPinpointDriver pinpoint;
    private final Telemetry telemetry; // NEW: Using the standard Driver Station telemetry tracker

    // UPDATED: Added Telemetry to the constructor so your OpModes can pass it in
    public Localization(HardwareMap hardwareMap, Telemetry telemetry) {
        this.telemetry = telemetry;

        // Connects to the hardware config name "pinpoint"
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        // Configures your 32mm gobilda 4-bar odometry pods
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        // Tells the computer where your pods are mounted (in mm from center of bot)
        pinpoint.setOffsets(-100.0, 100.0, DistanceUnit.INCH);

        // Sets encoder directions
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        // Zeroes out the map tracker
        pinpoint.resetPosAndIMU();
    }


    public void update() {
        // 1. Pulls the latest location from the pods
        pinpoint.update();

        // 2. Extracts coordinates (Fixed to use .getPose() which matches SDK 12)
        Pose2D pose = pinpoint.getPosition();
        double x = pose.getX(DistanceUnit.INCH);
        double y = pose.getY(DistanceUnit.INCH);
        double headingDeg = pose.getHeading(AngleUnit.DEGREES);

        // 3. Sends numbers directly to your Driver Station display instead of a dashboard map
        telemetry.addData("X Position (in)", x);
        telemetry.addData("Y Position (in)", y);
        telemetry.addData("Heading (deg)", headingDeg);
        telemetry.update();
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
