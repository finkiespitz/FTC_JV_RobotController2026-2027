package org.firstinspires.ftc.teamcode.common.Subsystems;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class DriveTrain {
    private final DcMotor frontLeft, frontRight, backLeft, backRight;
    private final IMU imu;

    private boolean slowMode = false;
    private boolean prevLeftStickButton = false;
    private boolean prevLeftAndRightBumper;

    public DriveTrain(HardwareMap hardwareMap) {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        imu = hardwareMap.get(IMU.class, "imu");
        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
                        RevHubOrientationOnRobot.UsbFacingDirection.UP
                )
        );
        imu.initialize(parameters);

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void resetHeading() {
        imu.resetYaw();
    }

    public void run(Gamepad gamepad1,
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
        double heading = -imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.RADIANS);
        double rotatedDrive = drive * Math.cos(heading) + strafe * Math.sin(heading);
        double rotatedStrafe = -drive * Math.sin(heading) + strafe * Math.cos(heading);

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
            resetHeading();
        }


        prevLeftAndRightBumper = gamepad1.left_bumper && gamepad1.right_bumper;


        telemetry.addData("Heading", heading);
        telemetry.update();
    }
}
