package org.firstinspires.ftc.teamcode.opmodes.auto;

import static com.pedropathing.ivy.Scheduler.isScheduled;
import static com.pedropathing.ivy.Scheduler.schedule;
import static com.pedropathing.ivy.commands.Commands.instant;
import static com.pedropathing.ivy.commands.Commands.waitMs;
import static com.pedropathing.ivy.commands.Commands.waitUntil;
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
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import com.pedropathing.ivy.commands.Commands;
import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.pedroPathing.Constants;
import org.firstinspires.ftc.teamcode.common.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.common.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.common.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.common.subsystems.NectarSensor;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.common.subsystems.Turret;
import org.firstinspires.ftc.teamcode.common.tuning.Localization;

@Autonomous(name = "Auto Blue1", group = "Autonomous")
public class AutoBlue extends LinearOpMode {

    private Follower follower;
    public Turret turret;
    public Intake Intake;
    public Limelight Limelight;
    public FlyWheel FlyWheel;
    public NectarSensor NectarSensor;
    public FlowerIntake FlowerIntake;
    public Transfer Transfer;
    private long shootTime = 400;



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
        FlyWheel.runAuto(Limelight.getPose(redTeam), NectarSensor.run(telemetry));
    });
    }


    private Command waitForFlywheelReady() {
        return waitUntil(() -> FlyWheel.isSpunUp());
    }
    private Command stopFlyWheel() {
        return instant(() -> {
            FlyWheel.stop();
        });
    }



    private Command shoot4Balls() {
        return sequential(
                instant(() -> Transfer.setPowerDirect(1)),
                waitMs(shootTime * 4)

        );
    }


    private Command intakeOn() {
        return instant(() -> {
            Intake.setPowerDirect(0.65);
        });
    }


    private Command intakeOff() {
        return instant(() -> {
            Intake.stop();
        });
    }



    private Command autoRoutine() {
        return sequential(
                spinUpFlywheel(),
                waitForFlywheelReady(),
                shoot4Balls(),
                stopFlyWheel(),

                parallel(
                        follow(follower, gardenIntake()),
                        intakeOn()
                ),

                intakeOff(),

                follow(follower, shoot2()),


                spinUpFlywheel(),
                waitForFlywheelReady(),
                shoot4Balls(),
                stopFlyWheel(),



                follow(follower, park()),

                intakeOff()
        );
    }


    @Override
    public void runOpMode() {
        this.NectarSensor = new NectarSensor(hardwareMap);
        this.Limelight = new Limelight(hardwareMap);
        this.FlyWheel = new FlyWheel(hardwareMap);
        this.turret = new Turret(hardwareMap);
        this.Intake = new Intake(hardwareMap);
        this.FlyWheel = new FlyWheel(hardwareMap);
        //this.FlowerIntake = new FlowerIntake(hardwareMap);
        this.Transfer = new Transfer (hardwareMap);

        follower = Constants.create(hardwareMap);

        follower.setPose(start);
        Scheduler.reset();

        telemetry.addLine("Auto Blue Full");
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

            telemetry.addData("Heading", Math.toDegrees(follower.pose().heading()));

            telemetry.addData("Mode", follower.mode());
            telemetry.addData("Completion", follower.completion());

            telemetry.update();
        }
    }
}
