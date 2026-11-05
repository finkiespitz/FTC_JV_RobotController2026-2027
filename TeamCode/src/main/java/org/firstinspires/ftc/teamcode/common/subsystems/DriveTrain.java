package org.firstinspires.ftc.teamcode.common.subsystems;

import android.annotation.SuppressLint;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.tuning.Localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DriveTrain {

    private final Localization localization;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private boolean slowMode = false;
    private boolean holdPose = false;
    private boolean prevLeftStickButton = false;
    private boolean prevRightTrigger = false;
    private boolean prevLeftAndRightBumper;
	
    private static double holdHeading = 0.0;
    private static double holdX = 0.0;
    private static double holdY = 0.0;
    private static double integralHeadingError = 0.0;
    private static double integralYError = 0.0;
    private static double integralXError = 0.0;
    private static double prevHeadingError = 0.0;
    private static double prevYError = 0.0;
    private static double prevXError = 0.0;
    public static double kpTwist = 0.0;
    public static double kpStrafe = 0.0;
    public static double kpDrive = 0.0;
    public static double kiTwist = 0.0;
    public static double kiStrafe = 0.0;
    public static double kiDrive = 0.0;
    public static double kdTwist = 0.0;
    public static double kdStrafe = 0.0;
    public static double kdDrive = 0.0;



    public double[] hiveLineUpValues(double tx, double targetYaw) {

        double strafe = tx * kpStrafe;
        double twist = targetYaw * kpTwist;

        return new double[]{strafe, twist};
    }

    @SuppressLint("SuspiciousIndentation")
    public double[] holdPoseValues(double y, double x, double headingDeg){
	//proportional error/ just base error
	double headingError = holdHeading - headingDeg;
	double yError = holdY - y;
        double xError = holdX - x;

	// Normalize heading error to always take the shortest path on a 0-359 scale
    	headingError = (headingError + 180) % 360;
    	if (headingError < 0) {
            headingError += 360;
   	    }
        headingError -= 180;
	
	//integral error
	integralHeadingError = integralHeadingError + headingError;
	integralYError = integralYError + yError;
	integralXError = integralXError + xError;

	//derivative error
	double derivativeHeadingError = headingError - prevHeadingError;
	double derivativeYError = yError - prevYError;
	double derivativeXError = xError - prevXError;

	//calculated outputs
	double twist = (headingError * kpTwist) + (integralHeadingError * kiTwist) + (derivativeHeadingError * kdTwist);
	double strafe = (yError * kpStrafe) + (integralYError * kiStrafe) + (derivativeYError * kdStrafe);
	double drive = (xError * kpDrive) + (integralXError * kiDrive) + (derivativeXError * kdDrive);

	//previous proportional/base errors
	prevHeadingError = headingError;
	prevYError = yError;
	prevXError = xError;

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
		holdHeading = headingDeg;
		holdX = x;
                holdY = y;

		integralHeadingError = 0;
		integralYError = 0;
 		integralXError = 0;
 		prevHeadingError = 0;
 		prevYError = 0;
 		prevXError = 0;
      
                holdPose = true;
	}

            prevRightTrigger = gamepad1.right_trigger_pressed;


            if(holdPose && !gamepad1.x) {
                double[] holdPoseValues = holdPoseValues(y, x, headingDeg);


                double holdStrafe = holdPoseValues[0];
 		twist = holdPoseValues[1];
		double holdDrive = holdPoseValues[2];
 		robotStrafe = (holdStrafe * Math.cos(headingRad)) - (holdDrive * Math.sin(headingRad)); 
		robotDrive = (holdStrafe * Math.sin(headingRad)) + (holdDrive * Math.cos(headingRad));

            }

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

        //reset heading controls

        if (gamepad1.left_bumper && gamepad1.right_bumper && !prevLeftAndRightBumper) {
            localization.resetHeading();
        }


        prevLeftAndRightBumper = gamepad1.left_bumper && gamepad1.right_bumper;
        
    }
}

