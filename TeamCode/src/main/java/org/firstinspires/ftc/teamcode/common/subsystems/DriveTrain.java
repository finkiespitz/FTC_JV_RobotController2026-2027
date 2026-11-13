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
    private final ElapsedTime dtTimer = new ElapsedTime();

    private boolean slowMode = false;
    private boolean holdPose = false;
    private boolean prevLeftStickButton = false;
    private boolean prevRightTrigger = false;
    private boolean prevLeftAndRightBumper = false;
    private boolean isFirstRun = true;
    private double dt = 0.01;

    private double holdHeadingDeg = 0.0;
    private double holdX = 0.0;
    private double holdY = 0.0;
    private double prevHeadingRad = 0.0;
    private double prevY = 0.0;
    private double prevX = 0.0;

    public static double kpTwist = 0.0;
    public static double kpStrafe = 0.0;


    public double[] hiveLineUpValues(double tx, double targetYaw) {

        double strafe = tx * kpStrafe;
        double twist = targetYaw * kpTwist;

        return new double[]{strafe, twist};
    }


    public double[] holdPoseValues(double y, double x, double headingDeg) {

        double xError = this.holdX - x;
        double yError = this.holdY - y;

        double headingErrorDeg = holdHeadingDeg - headingDeg;
        headingErrorDeg = ((headingErrorDeg + 180) % 360 + 360) % 360 - 180;

        double headingErrorRad = Math.toRadians(headingErrorDeg);
        double currentHeadingRad = Math.toRadians(headingDeg);

        if (this.isFirstRun) {
            this.prevX = x;
            this.prevY = y;
            this.prevHeadingRad = currentHeadingRad;
            this.isFirstRun = false;
            return new double[]{0, 0, 0};
        }

        if (this.dt < 0.001) {
            return new double[]{0, 0, 0};
        }

        double xVelocity = (x - this.prevX) / this.dt;
        double yVelocity = (y - this.prevY) / this.dt;

        double deltaHeading = currentHeadingRad - this.prevHeadingRad;
        double headingVelocity = Math.atan2(
                Math.sin(deltaHeading), Math.cos(deltaHeading)) / this.dt;

        this.prevX = x;
        this.prevY = y;
        this.prevHeadingRad = currentHeadingRad;

        double drive = -(xError * 94.86832981) - (xVelocity * 17.36981056);

        double strafe = -(yError * 100.00000000) - (yVelocity * 17.37247347);

        double twist = -(headingErrorRad * 180.27756377) - (headingVelocity * 31.80128451);

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


    public void run(Gamepad gamepad1, double headingRad, double tx, double targetYaw, double y, double x,
                    Telemetry telemetry) {
        this.dt = dtTimer.seconds();
        if (this.dt < 0.0001) this.dt = 0.01;
        dtTimer.reset();

        double headingDeg = Math.toDegrees(headingRad);
        // Toggle slow mode
        if (gamepad1.left_stick_button && !prevLeftStickButton) {
            slowMode = !slowMode;
        }
        prevLeftStickButton = gamepad1.left_stick_button;

        double drive = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;

        // Apply controller dead-zones
        if (Math.abs(drive) < 0.07) drive = 0;
        if (Math.abs(strafe) < 0.07) strafe = 0;
        if (Math.abs(twist) < 0.07) twist = 0;

        // Field-centric orientation geometry math
        double robotStrafe = (strafe * Math.cos(headingRad)) - (drive * Math.sin(headingRad));
        double robotDrive = (strafe * Math.sin(headingRad)) + (drive * Math.cos(headingRad));


       	if(gamepad1.right_trigger_pressed && !prevRightTrigger){
		    holdHeadingDeg = headingDeg;
		    holdX = x;
            holdY = y;
            isFirstRun = true;
            holdPose = !holdPose;

           }

            prevRightTrigger = gamepad1.right_trigger_pressed;

            //holdPose
            if(holdPose && !gamepad1.x) {
                double[] holdPoseValues = holdPoseValues(y, x, headingDeg);

                robotStrafe = holdPoseValues[0];
                twist = holdPoseValues[1];
                robotDrive = holdPoseValues[2];

            }

            //hive lineup
        if (gamepad1.x) {
            double[] hiveLineUpMovement =  hiveLineUpValues(tx, targetYaw);

            robotStrafe = hiveLineUpMovement[0];
            twist = hiveLineUpMovement[1];
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

        // Apply final speed profile multipliers
        double speedMultiplier = slowMode ? 0.4 : 1.0;
        frontLeft.setPower(speeds[0] * speedMultiplier);
        frontRight.setPower(speeds[1] * speedMultiplier);
        backLeft.setPower(speeds[2] * speedMultiplier);
        backRight.setPower(speeds[3] * speedMultiplier);

        
    }
}

