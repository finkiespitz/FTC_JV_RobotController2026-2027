package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.Robot;

@Config
@TeleOp(name = "MecanumDrive", group = "Linear OpMode")
public class TeleOpMecanumDrive extends LinearOpMode {


    private static double nectarFactor = 0.30;
    private static double pollenFactor = 0.10;


    @Override
    public void runOpMode() {
        // Instantiate the object-oriented master class
        Robot robot = new Robot(hardwareMap);

        telemetry.addData("Status", "Initialized Clean Robot Hub!");
        telemetry.update();

        waitForStart();
        robot.driveTrain.resetHeading();

        while (opModeIsActive()) {
            // 1. Run Drivetrain Subsystem
            robot.driveTrain.run(gamepad1);

            // 2. Run Intake Subsystem
            robot.Intake.run(gamepad1);

            // 3. Run FlyWheel Subsystem
            robot.FlyWheel.run(hardwareMap, nectarFactor, pollenFactor, gamepad1, gamepad2);

            //4. Run Flower Intake Subsystem
            robot.FlowerIntake.run(gamepad1, gamepad2
            );
        }
    }
}