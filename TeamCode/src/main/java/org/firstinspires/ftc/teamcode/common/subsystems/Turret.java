package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;


public class Turret {
    private boolean turretCentered = false;

    private final ElapsedTime centeringTimer = new ElapsedTime();

    private double centeringStartAngle = 0;

    private double zeroOffset = 0;

    private int hallDetectionCount = 0;

    private static final int HALL_DEBOUNCE_COUNT = 3;

    private static final double CENTERING_POWER = 0.08;
    private static final double CENTERING_TIMEOUT = 4.0;
    private static final double MAX_CENTERING_ROTATION = 120.0;
    private final ElapsedTime dtTimer = new ElapsedTime();
    private double turretAngle = 0;
    private double dt = 0.01;
    private double targetAngle = 0;
    private double currError = 0;
    private double kp = 0;
    private double ki = 0;
    private double kd = 0;
    private double prevError = 0;
    private double integral = 0;
    private double derivative = 0;
    private double currAngle = 0;

    private double headingDeg = 0.0;
    private boolean isAudienceSide;
    private final int GEAR_RATIO = 3;
    private static final double MAX_POWER = 0.65;
    private static final double INTEGRAL_LIMIT = 100.0;
    private static final double DERIVATIVE_FILTER = 0.8;
    private static final double POSITION_TOLERANCE = 1.0;
    private static final double HOMING_POWER = 0.08;
    private static final double HOMING_TIMEOUT = 4.0;
    private static final double MAX_HOMING_ROTATION = 120.0;
    private final CRServo turretServo;
    private DigitalChannel hallSensor;
    private double lastReading = 0.0;
    private int loopCount = 0;
    private AnalogInput axonFeedbackAnalog;
    private boolean magnetDetected = false;

    public Turret(HardwareMap hardwareMap) {
        turretServo = hardwareMap.get(CRServo.class, "turretServo");
        turretServo.setDirection(CRServo.Direction.FORWARD);

        axonFeedbackAnalog = hardwareMap.get(AnalogInput.class, "axonFeedback");

        hallSensor = hardwareMap.get(DigitalChannel.class, "hallSensor");
        hallSensor.setMode(DigitalChannel.Mode.INPUT);



    }


    public void updateTracking(double robotHeadingOffsetToHive) {
        if(turretCentered){
        targetAngle = getRawTurretAngle() - robotHeadingOffsetToHive;
        currAngle = getRawTurretAngle();
        currError = targetAngle - currAngle;
        integral = integral + (currError * dt );
        derivative = (currError - prevError) / dt;

        runPower((currError * kp) + (integral * ki) + (derivative * kd));
        prevError = currError;}
    }

    public void run(double robotHeadingOffsetToHive) {
        this.dt = dtTimer.seconds();
        if (this.dt < 0.0001) this.dt = 0.01;
        dtTimer.reset();
        magnetDetected = !hallSensor.getState();

        getServoAngle();
        centerTurret();
        updateTracking(robotHeadingOffsetToHive);
    }
    public double getServoAngle(){
        double currentReading = (axonFeedbackAnalog.getVoltage() / axonFeedbackAnalog.getMaxVoltage()) * 360.0;  // Maps to 0-360 degrees
        double turretDelta = currentReading - lastReading;

        // If it abruptly drops from 360 to 0 (spinning forward)
        if (turretDelta < -180) {
            loopCount++;
        }
        // If it abruptly jumps from 0 to 360 (spinning backward)
        else if (turretDelta > 180) {
            loopCount--;
        }

        double totalDegreesRotated = (loopCount * 360.0) + currentReading;
        lastReading = currentReading;
        turretAngle = totalDegreesRotated;
        return totalDegreesRotated;
    }

    public void centerTurret() {

        if (turretCentered) {
            turretServo.setPower(0);
            return;
        }

        // Timeout protection
        if (centeringTimer.seconds() > CENTERING_TIMEOUT) {
            turretServo.setPower(0);
            return;
        }

        // Maximum rotation protection
        double rotation = Math.abs(
                (turretAngle - centeringStartAngle) / GEAR_RATIO
        );

        if (rotation >= MAX_CENTERING_ROTATION) {
            turretServo.setPower(0);
            return;
        }

        // Hall sensor detection (active LOW)
        if (!hallSensor.getState()) {
            hallDetectionCount++;
        } else {
            hallDetectionCount = 0;
        }

        // Confirm magnet detection
        if (hallDetectionCount >= HALL_DEBOUNCE_COUNT) {

            turretServo.setPower(0);

            // Establish encoder zero
            zeroOffset = turretAngle;

            // Reset PID
            integral = 0;
            derivative = 0;
            prevError = 0;
            currError = 0;

            turretCentered = true;

            return;
        }

        // Continue rotating toward magnet
        turretServo.setPower(CENTERING_POWER);
    }


    public void runPower(double targetPower){
        turretServo.setPower(targetPower);}

    public double getRawTurretAngle(){return turretAngle / GEAR_RATIO;} //θ 2}
}
