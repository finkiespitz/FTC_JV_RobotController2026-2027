package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.tuning.Localization;


@TeleOp(name = "TeleOp", group = "Linear OpMode")
public class TeleOpBot1 extends LinearOpMode {


    private final double nectarFactor = 0.30;
    private final double pollenFactor = 0.10;


    @Override
    public void runOpMode() {
        // Instantiate the object-oriented master class
        Robot robot = new Robot(hardwareMap, telemetry);
        Localization localization = new Localization(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized Clean Robot Hub!");
        telemetry.update();

        waitForStart();
        robot.driveTrain.resetHeading();

        while (opModeIsActive()) {
            // 1. Run Drivetrain Subsystem
            robot.driveTrain.run(gamepad1,
                    telemetry);

            // 2. Run Intake Subsystem
            robot.Intake.run(gamepad2,
                    telemetry);

            // 3. Run FlyWheel Subsystem
            robot.FlyWheel.run(nectarFactor, pollenFactor, gamepad1, gamepad2,
                   robot.Limelight.getPose(), robot.NectarSensor.run(), telemetry);

            //4. Run Flower Intake Subsystem
            robot.FlowerIntake.run(gamepad2,
                    telemetry);

            localization.update();
        }
    }
}