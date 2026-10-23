package org.firstinspires.ftc.teamcode.common.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.tuning.Localization;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class DriveTrain {
    private Localization localization;

    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private boolean slowMode = false;
    private boolean prevLeftStickButton = false;
    private boolean prevLeftAndRightBumper;

    public DriveTrain(HardwareMap hardwareMap) {
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

    public void run(Gamepad gamepad1, double headingRad,
                    Telemetry telemetry) {


        // Toggle slow mode
        if (gamepad1.left_stick_button && !prevLeftStickButton) {
            slowMode = !slowMode;
        }
        prevLeftStickButton = gamepad1.left_stick_button;

        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;

        // Apply controller dead-zones
        if (Math.abs(drive) < 0.07) drive = 0;
        if (Math.abs(strafe) < 0.07) strafe = 0;
        if (Math.abs(twist) < 0.07) twist = 0;

        // Field-centric orientation geometry math
        double rotatedDrive = drive * Math.cos(headingRad) + strafe * Math.sin(headingRad);
        double rotatedStrafe = -drive * Math.sin(headingRad) + strafe * Math.cos(headingRad);

        // Map drive vectors to power arrays
        double[] speeds = {
                (rotatedDrive + rotatedStrafe + twist),
                (rotatedDrive - rotatedStrafe - twist),
                (rotatedDrive - rotatedStrafe + twist),
                (rotatedDrive + rotatedStrafe - twist)
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

        telemetry.update();
    }
}
