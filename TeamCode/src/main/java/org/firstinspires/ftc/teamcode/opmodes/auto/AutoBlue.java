package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.Scheduler.isScheduled;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.groups.Groups.parallel;
import static com.pedropathing.ivy.groups.Groups.sequential;
import static com.pedropathing.ivy.pedro.PedroCommands.follow;

import com.pedropathing.api.Paths;
import com.pedropathing.api.PoseFactory;
import com.pedropathing.follower.Follower;
import com.pedropathing.ivy.Command;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.interpolator.Interpolator;
import com.pedropathing.utils.Timer; // Plural version verified for your project configuration
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.pedroPathing.Constants;

@Autonomous(name = "Auto Blue1", group = "Autonomous")
public class AutoBlue extends LinearOpMode {

    private Follower follower;
    private Robot robot;

    private final boolean redTeam = false;

    private final PoseFactory poseFactory = PoseFactory.degrees().mirrorX(70.75).mirrorY(70.75);
    private final Pose start = poseFactory.of(56, 8, 90);
    private final Pose gardenIntake = poseFactory.of(8, 8, 270);
    private final Pose gardenIntakeControl1 = poseFactory.of(60.2001, 20.8639, 0);
    private final Pose gardenIntakeControl2 = poseFactory.of(40.8954, 72.1897, 0);
    private final Pose shoot2 = poseFactory.of(60, 133, 270);
    private final Pose shoot2Control1 = poseFactory.of(33.6528, 47.5302, 0);
    private final Pose shoot2Control2 = poseFactory.of(1.4226, 126.9303, 0);
    private final Pose shoot2Control3 = poseFactory.of(60.0322, 106.5034, 0);
    private final Pose shoot2Segment1Heading = poseFactory.of(60, 133, 270);
    private final Pose shoot2Segment2Start = poseFactory.of(60, 133, 270);
    private final Pose shoot2Segment2End = poseFactory.of(60, 133, 0);
    private final Pose shoot2Segment3Start = poseFactory.of(60, 133, 0);
    private final Pose shoot2Segment3End = poseFactory.of(60, 133, 270);
    private final Pose parkStart = poseFactory.of(60, 133, 270);
    private final Pose park = poseFactory.of(15.7594, 119.9135, 0);
    private final Pose parkControl1 = poseFactory.of(62.6887, 104.738, 0);

    public Path gardenIntake() {return Paths.curve(start, gardenIntakeControl1, gardenIntakeControl2, gardenIntake).linear(start, gardenIntake);}


    public Path shoot2() {
        return Paths.curve(
                gardenIntake, shoot2Control1, shoot2Control2, shoot2Control3, shoot2).heading(
                Interpolator.piecewise().until(0.0507,
                        Interpolator.constant(shoot2Segment1Heading)).until(0.3524,
                        Interpolator.linear(
                                shoot2Segment2Start,
                                shoot2Segment2End)).until(1,
                        Interpolator.linear(
                                shoot2Segment3Start,
                                shoot2Segment3End))
        );
    }


    public Path park() {return Paths.curve(parkStart, parkControl1, park).linear(parkStart, park);}


    private Command spinUpFlywheel() {return instant(() -> {
        // Flywheel on
    });
    }


    private Command waitForFlywheelReady() {
        return instant(() -> {
            // Wait for flywheel speed
        });
    }


    private Command shootPreloads() {
        return instant(() -> {
            // Shoot preload
        });
    }


    private Command intakeOn() {
        return instant(() -> {
            // Intake on
        });
    }


    private Command intakeOff() {
        return instant(() -> {
            // Intake off
        });
    }


    private Command transferRings() {
        return instant(() -> {
            // Transfer rings
        });
    }


    private Command shootSecondVolley() {
        return instant(() -> {
            // Shoot second volley
        });
    }


    private Command autoRoutine() {
        return sequential(
                spinUpFlywheel(),
                waitForFlywheelReady(),
                shootPreloads(),

                parallel(
                        follow(follower, gardenIntake()),
                        intakeOn()
                ),

                intakeOff(),

                follow(follower, shoot2()),

                parallel(
                        spinUpFlywheel(),
                        transferRings()
                ),

                shootSecondVolley(),

                follow(follower, park()),

                intakeOff()
        );
    }


    @Override
    public void runOpMode() {

        follower = Constants.create(hardwareMap);

        follower.setPose(start);
        Scheduler.reset();

        telemetry.addLine("Auto Red Full");
        telemetry.addData("Start X", start.x());
        telemetry.addData("Start Y", start.y());
        telemetry.addData("Start Heading", start.heading());
        telemetry.update();

        waitForStart();

        if (isStopRequested()) return;

        schedule(autoRoutine());

        while (opModeIsActive() && !isStopRequested()) {

            follower.update();
            Scheduler.execute();

            telemetry.addData("X", follower.pose().x());
            telemetry.addData("Y", follower.pose().y());

            telemetry.addData(
                    "Heading",
                    Math.toDegrees(follower.pose().heading())
            );

            telemetry.addData("Mode", follower.mode());
            telemetry.addData("Completion", follower.completion());

            telemetry.update();
        }
    }
}
