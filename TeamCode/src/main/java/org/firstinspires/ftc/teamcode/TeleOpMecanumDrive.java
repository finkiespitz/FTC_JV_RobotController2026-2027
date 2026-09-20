package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

@TeleOp(name = "MecanumDrive", group = "Linear OpMode")
public class TeleOpMecanumDrive extends LinearOpMode {

    private DcMotor frontLeft;
    private DcMotor frontRight;
    private DcMotor backLeft;
    private DcMotor backRight;
    private DcMotor intakeMotor;

    private IMU imu;

    private boolean slowMode = false;
    private boolean previousLeftStickButton = false;
    private boolean intakeActive = false;


    @Override
    public void runOpMode() {

        // Initialize hardware here (e.g., hardwareMap.get(DcMotor.class, "motorName"));

        //Defining the motors.
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        //Defining the servos.


        //Defining sensors
        imu = hardwareMap.get(IMU.class, "imu");

        IMU.Parameters parameters = new IMU.Parameters(
                new com.qualcomm.hardware.rev.RevHubOrientationOnRobot(
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection.UP,
                        com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection.FORWARD
                )
        );
        imu.initialize(parameters);


        //Setting the direction of the motors.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);

        //Motor ZeroPower b  ehavior.
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //More servo settings.

        telemetry.addData("Status", "Initialized Field Oriented Mecanum Drive");
        telemetry.update();


        waitForStart();
        //Reset heading when program starts.
        imu.resetYaw();


        //Main control loop
        while (opModeIsActive()) {



            /*==========================================*/
            /*             Mecanum Drive                */
            /*==========================================*/


            // Mecanum drive is controlled with three axes: drive (front-and-back),
            // strafe (left-and-right), and twist (rotating the whole chassis).

            //Joystick inputs
            double drive = -gamepad2.left_stick_y;
            double strafe = gamepad2.left_stick_x;
            double twist = gamepad2.right_stick_x;


            //This is a toggle between slow mode and fast mode
            //using the left stick button (but can be changed to anything)

            if (gamepad2.left_stick_button && !previousLeftStickButton) {
                slowMode = !slowMode;
            }
            previousLeftStickButton = gamepad2.left_stick_button;



            if (Math.abs(drive) < 0.05) drive = 0;
            if (Math.abs(strafe) < 0.05) strafe = 0;
            if (Math.abs(twist) < 0.05) twist = 0;


            //Field centric control//

            // Get robot heading in radians.
            double heading = -imu.getRobotYawPitchRollAngles()
                    .getYaw(AngleUnit.RADIANS);

            double rotatedDrive =
                    drive * Math.cos(heading)
                            + strafe * Math.sin(heading);

            double rotatedStrafe =
                    -drive * Math.sin(heading)
                            + strafe * Math.cos(heading);

            // Use the rotated values for mecanum calculations.
            drive = rotatedDrive;
            strafe = rotatedStrafe;


            //Set the speed for each motor.
            double[] speeds = {
                    (drive + strafe + twist),
                    (drive - strafe - twist),
                    (drive - strafe + twist),
                    (drive + strafe - twist)
            };

            // Because we are adding vectors and motors only take values between
            // [-1,1] we may need to normalize them.

            // Loop through all values in the speeds[] array and find the greatest
            // *magnitude*.  Not the greatest velocity.

            double max = Math.abs(speeds[0]);
            for (double speed : speeds) {
                if (max < Math.abs(speed)) max = Math.abs(speed);
            }

            // If and only if the maximum is outside the range we want it to be,
            // normalize all the other speeds based on the given speed value.

            if (max > 1) {
                for (int i = 0; i < speeds.length; i++) speeds[i] /= max;
            }

            //Applying the speed multiplier to the motor calculations.
            //Can be changed to anything from 100%(1.0) down to 10%(0.1).

            double speedMultiplier = slowMode ? 0.4 : 1.0;

            for (int i = 0; i < speeds.length; i++) {
                speeds[i] *= speedMultiplier;
            }


            // apply the calculated values to the motors.
            frontLeft.setPower(speeds[0]);
            frontRight.setPower(speeds[1]);
            backLeft.setPower(speeds[2]);
            backRight.setPower(speeds[3]);


            /*==========================================*/
            /*                 Intake                   */
            /*==========================================*/

            //checking if the bumper is pressed

            if (gamepad2.left_bumper){
                intakeActive = true;}

            else {
                intakeActive = false;}

            //setting the power to the intakeMotor
            //you can change it from 0.1 - 1

            double intakePower = intakeActive ? 1 : 0;

            intakeMotor.setPower(intakePower);


            //All the telemetry goes here.
            telemetry.addData("Heading", "%.1f degrees", Math.toDegrees(heading));
            telemetry.addData("Drive", "FL %.2f | FR %.2f", speeds[0], speeds[1]);
            telemetry.addData("", "BL %.2f | BR %.2f", speeds[2], speeds[3]);

            telemetry.addData("Slow Mode", slowMode ? "ON (40%)" : "OFF (100%)");
            telemetry.update();
        }
    }
}


