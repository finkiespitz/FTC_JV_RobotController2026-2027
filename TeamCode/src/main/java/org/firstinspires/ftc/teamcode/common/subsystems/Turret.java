
package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Turret {

    private boolean turretCentered = false;
    private boolean homingFailed = false;


    private final ElapsedTime centeringTimer = new ElapsedTime();
    private final ElapsedTime dtTimer = new ElapsedTime();

    private double centeringStartAngle = 0;
    private double zeroOffset = 0;

    private int hallDetectionCount = 0;

    private static final int HALL_DEBOUNCE_COUNT = 3;
    private static final double CENTERING_POWER = 0.08;
    private static final double CENTERING_TIMEOUT = 4.0;
    private static final double MAX_CENTERING_ROTATION = 120.0;

    private double turretAngle = 0;
    private double dt = 0.01;

    private double targetAngle = 0;
    private double currError = 0;

    public static double kp = 0;
    public static double ki = 0;
    public static double kd = 0;

    private double prevError = 0;
    private double integral = 0;
    private double derivative = 0;
    private double currAngle = 0;

    private static final int GEAR_RATIO = 3;
    private static final double MAX_POWER = 0.65;
    private static final double INTEGRAL_LIMIT = 100.0;

    private final CRServo turretServo;
    private final DigitalChannel hallSensor;
    private final AnalogInput axonFeedbackAnalog;

    private double lastReading = 0.0;
    private int loopCount = 0;

    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        turretServo.setDirection(CRServo.Direction.FORWARD);

        axonFeedbackAnalog = hardwareMap.get(AnalogInput.class, "axonFeedback");

        hallSensor = hardwareMap.get(DigitalChannel.class, "hallSensor");
        hallSensor.setMode(DigitalChannel.Mode.INPUT);

        centeringTimer.reset();
        dtTimer.reset();
    }

    public void updateTracking(double targetRobotHeading, double robotHeadingDeg) {
        if (turretCentered && !homingFailed) {

            targetAngle = targetRobotHeading;

            targetAngle = ((targetAngle + 180) % 360 + 360) % 360 - 180;

            currAngle = getAbsoluteTurretHeadingDeg(robotHeadingDeg);

            currError = targetAngle - currAngle;

            currError = ((currError + 180) % 360 + 360) % 360 - 180;

            integral += currError * dt;
            integral = Math.max(-INTEGRAL_LIMIT, Math.min(INTEGRAL_LIMIT, integral));

            derivative = (currError - prevError) / dt;

            double power = (currError * kp) + (integral * ki) + (derivative * kd);

            power = Math.max(-MAX_POWER, Math.min(MAX_POWER, power));

            runPower(power);

            prevError = currError;
        }
    }

    public void run(double robotHeadingOffsetToHive, double robotHeadingDeg) {
        dt = dtTimer.seconds();

        if (dt < 0.0001) dt = 0.01;

        dtTimer.reset();

        getServoAngle();

        centerTurret();

        updateTracking(robotHeadingOffsetToHive, robotHeadingDeg);
    }

    public double getServoAngle() {
        double currentReading = (axonFeedbackAnalog.getVoltage() / axonFeedbackAnalog.getMaxVoltage()) * 360.0;
        double turretDelta = currentReading - lastReading;

        if (turretDelta < -180) {
            loopCount++;
        } else if (turretDelta > 180) {
            loopCount--;
        }

        double totalDegreesRotated = (loopCount * 360.0) + currentReading;

        lastReading = currentReading;
        turretAngle = totalDegreesRotated;

        return totalDegreesRotated;
    }

    public void centerTurret() {

        if (turretCentered || homingFailed) {
            turretServo.setPower(0);
            return;
        }

        if (centeringTimer.seconds() > CENTERING_TIMEOUT) {
            homingFailed = true;
            turretServo.setPower(0);
            return;
        }

        double rotation = Math.abs(
                (turretAngle - centeringStartAngle) / GEAR_RATIO
        );

        if (rotation >= MAX_CENTERING_ROTATION) {
            homingFailed = true;
            turretServo.setPower(0);
            return;
        }

        if (!hallSensor.getState()) {
            hallDetectionCount++;
        } else {
            hallDetectionCount = 0;
        }

        if (hallDetectionCount >= HALL_DEBOUNCE_COUNT) {

            turretServo.setPower(0);

            zeroOffset = turretAngle;

            integral = 0;
            derivative = 0;
            prevError = 0;
            currError = 0;

            turretCentered = true;

            return;
        }

        turretServo.setPower(CENTERING_POWER);
    }

    public void runPower(double targetPower) {
        turretServo.setPower(targetPower);
    }

    public double getRawTurretAngle() {return (getServoAngle() - zeroOffset) / GEAR_RATIO;}
    public double getAbsoluteTurretHeadingDeg(double robotHeadingDeg){return getRawTurretAngle() - robotHeadingDeg;}

    public boolean isTurretCentered() {
        return turretCentered;
    }

    public boolean hasHomingFailed() {
        return homingFailed;
    }
}
