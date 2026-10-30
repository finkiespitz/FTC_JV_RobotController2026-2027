package org.firstinspires.ftc.teamcode.opmodes.teleOp;

import org.firstinspires.ftc.teamcode.common.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "TeleOpRed", group = "Linear OpMode")
public class TeleOpBot1Red extends LinearOpMode {
    boolean redTeam = true;

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Instantiate the object-oriented master class
        Robot robot = new Robot(hardwareMap, telemetry);

        telemetry.addData("Status", "Initialized Clean Robot Hub!");
        telemetry.update();

        waitForStart();
        robot.Localization.resetHeading();

        while (opModeIsActive()) {

            ///update localization
            robot.Localization.update();

            // 1. Run Drivetrain Subsystem
            robot.driveTrain.run(gamepad1, robot.Localization.getHeadingRad(), redTeam, robot.Limelight.getTx(redTeam),
                    telemetry);

            // 2. Run Intake Subsystem
            robot.Intake.run(gamepad2,
                    telemetry);

            // 3. Run FlyWheel Subsystem
            robot.FlyWheel.run(gamepad1, gamepad2, robot.Limelight.getPose(redTeam),
                    robot.NectarSensor.run(telemetry), telemetry);

            //4. Run Flower Intake Subsystem
            //robot.FlowerIntake.run(gamepad2,
            //        telemetry);

            //5. Run Transfer Subsystem
            robot.Transfer.run(gamepad2,
                    telemetry);

            telemetry.update();
        }
    }
}