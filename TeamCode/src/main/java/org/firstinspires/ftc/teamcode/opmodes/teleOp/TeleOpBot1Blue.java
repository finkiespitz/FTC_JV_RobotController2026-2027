package org.firstinspires.ftc.teamcode.opmodes.teleOp;
import org.firstinspires.ftc.teamcode.common.Robot;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;

@Config
@TeleOp(name = "TeleOpBlue", group = "Linear OpMode")
public class TeleOpBot1Blue extends LinearOpMode {
    boolean redTeam = false;

    @Override
    public void runOpMode() {

        FtcDashboard dashboard = FtcDashboard.getInstance();

        // Instantiate the object-oriented master class
        Robot robot = new Robot(hardwareMap, telemetry, redTeam);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        robot.Localization.resetHeading();

        while (opModeIsActive()) {

            //update localization
            robot.Localization.update();
	        robot.Localization.correctHeadingWithVision(robot.Limelight, redTeam);

            //run Drivetrain Subsystem
            robot.driveTrain.run(gamepad1, robot.Localization.getHeadingRad(), robot.Limelight.getTx(redTeam), robot.Limelight.getTargetYaw(redTeam), robot.Localization.getY(), robot.Localization.getX(),
                    telemetry);

            robot.turret.run(gamepad2, robot.Localization.getX(), robot.Localization.getY(), redTeam,
                    telemetry);

            //  Run Intake Subsystem
            robot.Intake.run(gamepad2,
                    telemetry);

            //  Run FlyWheel Subsystem
            robot.FlyWheel.run(gamepad1, gamepad2, robot.Limelight.getPose(redTeam),
                    robot.NectarSensor.run(telemetry), telemetry);

            // Run Flower Intake Subsystem
            //robot.FlowerIntake.run(gamepad2,
            //        telemetry);

            // Run Transfer Subsystem
            robot.Transfer.run(gamepad2,
                    telemetry);

            telemetry.update();
        }
    }
}