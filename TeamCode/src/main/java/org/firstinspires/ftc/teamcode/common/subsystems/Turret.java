package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Turret {
    private final Servo turretServo;
    private double targetX = 0.0;
    private double targetY = 0.0;
    private double turretAngle = 0.0;

    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(Servo.class, "turretServo");
        turretServo.setPosition(0.0);
    }

    public void calculateHiveTarget(double currentY, boolean redTeam) {
        boolean isAudienceSide = currentY < 70.75;

        if (redTeam) {
            targetX = 57.7;
            targetY = isAudienceSide ? 86.5 : 55.0;
        } else {
            targetX = 83.8;
            targetY = isAudienceSide ? 86.7 : 55.0;
        }
    }

    public void updateTracking(double currentX, double currentY, double headingDeg) {
        double deltaX = targetX - currentX;
        double deltaY = targetY - currentY;


        double absoluteFieldAngle = Math.toDegrees(Math.atan2(deltaY, deltaX));

        double localTargetAngle = absoluteFieldAngle - headingDeg;


        double rawTurretAngle = 90.0 - localTargetAngle;


        rawTurretAngle = (rawTurretAngle + 180) % 360;
        if (rawTurretAngle < 0) {
            rawTurretAngle += 360;
        }
        rawTurretAngle -= 180;


        if (rawTurretAngle < 0.0) {
            turretAngle = 0.0;
        } else {
            turretAngle = Math.min(rawTurretAngle, 180.0);
        }
    }

    public void run(double currentX, double currentY, double currentHeadingDeg, boolean redTeam, Telemetry telemetry) {
        calculateHiveTarget(currentY, redTeam);
        updateTracking(currentX, currentY, currentHeadingDeg);

        double servoPose = turretAngle / 180.0;
        turretServo.setPosition(servoPose);

        telemetry.addData("Turret Target X", targetX);
        telemetry.addData("Turret Target Y", targetY);
        telemetry.addData("Turret Angle Deg", turretAngle);
        telemetry.addData("Turret Servo Pose", turretServo.getPosition());
    }

    public void setPose(double pose) {
        turretServo.setPosition(pose);
    }

    public double getTargetX() { return targetX; }
    public double getTargetY() { return targetY; }
    public double getTurretAngle() { return turretAngle; }
}