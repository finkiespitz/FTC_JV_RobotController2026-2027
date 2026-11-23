package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.tuning.Localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class DriveTrain {

    private final Localization localization;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;

    private boolean holdPose = false;
    private double holdY = 0.0;
    private double holdX = 0.0;
    private double holdHeadingDeg = 0.0;
    private double prevXError = 0;
    private double prevYError = 0;
    private double prevHeadingError = 0;
    private double driveIntegral = 0;
    private double strafeIntegral = 0;
    private double twistIntegral = 0;

    private final ElapsedTime pidTimer = new ElapsedTime();
    private double dt = 0.01;

    public static double kpDrive = 0;
    public static double kiDrive = 0;
    public static double kdDrive = 0;

    public static double kpStrafe = 0;
    public static double kiStrafe = 0;
    public static double kdStrafe = 0;

    public static double kpTwist = 0;
    public static double kiTwist = 0;
    public static double kdTwist = 0;




    public double[] hiveLineUpValues(double hiveTargetHeading, double heading) {
        double error = hiveTargetHeading - heading;
        double twist = error * kpTwist;

        return new double[]{twist};
    }


    public double[] holdPoseValues(double y, double x, double headingDeg) {
        dt = pidTimer.seconds();
        pidTimer.reset();
        if (dt < 0.001) dt = 0.01;

        double xError = holdX - x;
        double yError = holdY - y;

        double headingErrorDeg = holdHeadingDeg - headingDeg;
        headingErrorDeg = ((headingErrorDeg + 180) % 360 + 360) % 360 - 180;

// ================= DRIVE PID =================

        driveIntegral += xError * dt;
        double driveDerivative = (xError - prevXError) / dt;

        double drive = kpDrive * xError + kiDrive * driveIntegral + kdDrive * driveDerivative;

// ================= STRAFE PID =================

        strafeIntegral += yError * dt;
        double strafeDerivative = (yError - prevYError) / dt;

        double strafe = kpStrafe * yError + kiStrafe * strafeIntegral + kdStrafe * strafeDerivative;

// ================= TWIST PID =================

        twistIntegral += headingErrorDeg * dt;
        double twistDerivative = (headingErrorDeg - prevHeadingError) / dt;

        double twist =
                kpTwist * headingErrorDeg + kiTwist * twistIntegral + kdTwist * twistDerivative;

// Update previous errors
        prevXError = xError;
        prevYError = yError;
        prevHeadingError = headingErrorDeg;

        return new double[]{strafe, twist, drive};
    }


    public DriveTrain(Localization localization, HardwareMap hardwareMap) {
        this.localization = localization;
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");


        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void run(Gamepad gamepad1, double headingRad, double tx, double targetYaw, double y, double x, boolean redTeam,
                    Telemetry telemetry) {

        double headingDeg = Math.toDegrees(headingRad);


        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;

        // Apply controller dead-zones
        if (Math.abs(drive) < 0.07) drive = 0;
        if (Math.abs(strafe) < 0.07) strafe = 0;
        if (Math.abs(twist) < 0.07) twist = 0;

        // Field-centric orientation geometry math
        double robotStrafe = (strafe * Math.cos(headingRad)) + (drive * Math.sin(headingRad));
        double robotDrive = -(strafe * Math.sin(headingRad)) + (drive * Math.cos(headingRad));


        if(gamepad1.touchpadWasPressed()){
            holdHeadingDeg = headingDeg;
            holdX = x;
            holdY = y;
            holdPose = !holdPose;
            prevXError = 0;
            prevYError = 0;
            prevHeadingError = 0;
            driveIntegral = 0;
            strafeIntegral = 0;
            twistIntegral = 0;
            pidTimer.reset();

        }



        //holdPose
        if(holdPose && !gamepad1.x) {
            double[] holdPoseValues = holdPoseValues(y, x, headingDeg);

            robotStrafe = holdPoseValues[0];
            twist = holdPoseValues[1];
            robotDrive = holdPoseValues[2];

        }

        //hive lineup
        if (gamepad1.x) {double[] hiveLineUpMovement =  hiveLineUpValues(localization.getRobotHeadingOffsetToHive(redTeam, x, y), headingDeg);

            twist = hiveLineUpMovement[0];
            robotDrive = 0;
        }

        // Map drive vectors to power arrays
        double[] speeds = {
                (robotDrive + robotStrafe + twist), // Front Left
                (robotDrive - robotStrafe - twist), // Front Right
                (robotDrive - robotStrafe + twist), // Back Left
                (robotDrive + robotStrafe - twist)  // Back Right
        };


        // Normalize if combined motor vectors break maximum limits
        double max = Math.abs(speeds[0]);
        for (double speed : speeds) {
            if (max < Math.abs(speed)) max = Math.abs(speed);
        }
        if (max > 1) {
            for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
        }

        frontLeft.setPower(speeds[0]);
        frontRight.setPower(speeds[1]);
        backLeft.setPower(speeds[2]);
        backRight.setPower(speeds[3]);


    }
}