package org.firstinspires.ftc.teamcode.opmodes.teleOp;
import org.firstinspires.ftc.teamcode.common.Robot;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "TeleOpBlue", group = "Linear OpMode")
public class TeleOpBot1Blue extends LinearOpMode {
    private boolean blueTeam = true;
    @Override
    public void runOpMode() {
        // Instantiate the object-oriented master class
        Robot robot = new Robot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized Clean Robot Hub!");
        telemetry.update();

        waitForStart();
        robot.Localization.resetHeading();

        while (opModeIsActive()) {
            // 1. Run Drivetrain Subsystem
            robot.driveTrain.run(gamepad1, robot.Localization.getHeadingRad(),
                    telemetry);

            // 2. Run Intake Subsystem
            robot.Intake.run(gamepad2,
                    telemetry);

             // 3. Run FlyWheel Subsystem
            robot.FlyWheel.run(gamepad1, gamepad2, robot.Limelight.getPoseBlue(),
                    robot.NectarSensor.run(), telemetry);

            //4. Run Flower Intake Subsystem
            robot.FlowerIntake.run(gamepad2,
                    telemetry);

            robot.Localization.update();
        }
    }
}