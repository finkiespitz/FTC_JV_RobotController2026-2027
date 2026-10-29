package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.tuning.Localization;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Config
public class DriveTrain {

    private final Localization localization;
    private final DcMotor frontLeft, frontRight, backLeft, backRight;


    private boolean slowMode = false;
    private boolean prevLeftStickButton = false;
    private boolean prevLeftAndRightBumper;
    private static double kpTurn = 1.5;
    private static double kpStrafe = 0.04;


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

    public void run(Gamepad gamepad1, double headingRad, boolean redTeam, double Tx,
                    Telemetry telemetry) {


        // Toggle slow mode
        if (gamepad1.left_stick_button && !prevLeftStickButton) {
            slowMode = !slowMode;
        }
        prevLeftStickButton = gamepad1.left_stick_button;

        double drive = gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double twist = gamepad1.right_stick_x;

        if (gamepad1.x) {
            // 1. Lock Heading Angle (IMU-based)
            double targetHeading = redTeam ? Math.PI : 0.0;
            double headingError = targetHeading - headingRad;
            while (headingError > Math.PI) headingError -= 2 * Math.PI;
            while (headingError < -Math.PI) headingError += 2 * Math.PI;

            twist = headingError * kpTurn;
            twist = Math.max(-0.6, Math.min(0.6, twist)); // Clamp rotation to prevent spinning too wildly

            // 2. Lock Strafe Centering (Limelight TX crosshair error-based)

            // Only adjust strafe if Limelight is seeing a verified target tag
            if (Math.abs(Tx) != 0) {
                strafe = Tx * kpStrafe;
                strafe = Math.max(-0.5, Math.min(0.5, strafe)); // Clamp slide speed to safely slide across the target
                telemetry.addData("[Auto-Align]", "Tag Locked! Horizontal Offset: %.2f deg | Strafe Power: %.2f", Tx, strafe);
            } else {
                telemetry.addData("[Auto-Align]", "Tag detected but centered or invalid. Searching...");
            }
        }

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

    }
}