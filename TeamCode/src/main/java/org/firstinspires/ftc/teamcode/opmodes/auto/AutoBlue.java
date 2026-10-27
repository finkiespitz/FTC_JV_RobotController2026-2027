package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer; // Plural version verified for your project configuration
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pedroPaths.PedroPathBlue1;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pedroPaths.PedroPathRed1;

@Autonomous(name = "Auto Blue", group = "Autonomous")
public class AutoBlue extends LinearOpMode {

    private Follower follower;
    private Robot robot;
    private PedroPathBlue1 paths;
    private Timer stateTimer;

    // Set to true to filter the Red alliance Hive tags correctly during autonomous shooting
    private final boolean redTeam = false;

    // The states our autonomous machine will step through sequentially
    private enum AutoState {
        SHOOT_PRELOADS,
        DRIVE_TO_GARDEN,
        DRIVE_TO_HIVE,
        SHOOT_GARDEN_PIECES,
        PARK,
        DONE
    }

    private AutoState currentState = AutoState.SHOOT_PRELOADS;

    @Override
    public void runOpMode() {
        //initialization
        follower = org.firstinspires.ftc.teamcode.common.pedroPathing.Constants.create(hardwareMap);
        robot = new Robot(hardwareMap, telemetry);
        paths = new PedroPathBlue1();
        stateTimer = new Timer();

        telemetry.addData("Status", "Auto Initialized");
        telemetry.update();

        waitForStart();

        robot.Localization.resetHeading();
        setNextState(AutoState.SHOOT_PRELOADS);

        while (opModeIsActive() && !isStopRequested()) {
            // Keep background odometry and path algorithms ticking
            follower.update();
            robot.Localization.update();

            // Extract native milliseconds from Pedro's timer and declare elapsedSeconds decimal safely
            double elapsedMillis = stateTimer.milliseconds();
            double elapsedSeconds = elapsedMillis / 1000.0;

            switch (currentState) {

                case SHOOT_PRELOADS:
                    // Auto-shoots the 4 preloaded pieces first
                    runAutoShooterSequence(4, (long) elapsedMillis, AutoState.DRIVE_TO_GARDEN);
                    break;

                case DRIVE_TO_GARDEN:
                    // One-time execution trigger block to start path sequence
                    if (elapsedSeconds < 0.1) {
                        follower.follow(paths.gardenIntake());
                        robot.Intake.setPowerDirect(1.0); // Spin vacuum intake wheels forward
                    }

                    // Move to next state once robot arrives at destination coordinates
                    if (!follower.isBusy()) {
                        setNextState(AutoState.DRIVE_TO_HIVE);
                    }
                    break;

                case DRIVE_TO_HIVE:
                    // One-time execution trigger block to start path sequence
                    if (elapsedSeconds < 0.1) {
                        follower.follow(paths.shoot2());
                    }

                    // HALFWAY CUTOFF CHECK: Stop intake at 50% path line completion progress
                    if (follower.completion() >= 0.50) {
                        robot.Intake.setPowerDirect(0.0);
                    }

                    // Arrived at the Hive shooting station
                    if (!follower.isBusy()) {
                        robot.Intake.setPowerDirect(0.0); // Safety fallback cutoff
                        setNextState(AutoState.SHOOT_GARDEN_PIECES);
                    }
                    break;

                case SHOOT_GARDEN_PIECES:
                    // Auto-shoots the 4 gathered pieces from the garden
                    runAutoShooterSequence(4, (long) elapsedMillis, AutoState.PARK);
                    break;

                case PARK:
                    // One-time execution trigger block to start path sequence
                    if (elapsedSeconds < 0.1) {
                        follower.follow(paths.park());
                    }

                    if (!follower.isBusy()) {
                        setNextState(AutoState.DONE);
                    }
                    break;

                case DONE:
                    // Explicitly isolate power channels to prevent motor stall heating
                    robot.FlyWheel.start(0);
                    robot.Transfer.setPowerDirect(0);
                    robot.Intake.setPowerDirect(0);
                    break;
            }

            // Realtime telemetry metrics tracking dashboard on the controller phone screen
            telemetry.addData("Current State", currentState);
            telemetry.addData("State Time (sec)", elapsedSeconds);
            telemetry.addData("Path Completion T", follower.completion());
            telemetry.update();
        }
    }

    /**
     * Resets state timers cleanly between action shifts.
     */
    private void setNextState(AutoState nextState) {
        currentState = nextState;
        stateTimer.reset();
    }

    /**
     * Automates individual game-piece firing cycles dynamically.
     */
    private void runAutoShooterSequence(int targetCount, long currentMillis, AutoState nextState) {
        double currentDistance = robot.Limelight.getPose(redTeam);
        boolean nectarIsNext = robot.NectarSensor.run(telemetry);

        long singleShotTime = 700; // 350ms spool-up + 350ms feed window duration
        long totalSequenceTimeNeeded = targetCount * singleShotTime;

        if (currentMillis < totalSequenceTimeNeeded) {
            long timeIntoCurrentShot = currentMillis % singleShotTime;

            // Phase A: Rev shooter wheels up using distance maps (0ms to 350ms)
            if (timeIntoCurrentShot < 350) {
                robot.FlyWheel.run(gamepad1, gamepad2, currentDistance, nectarIsNext, telemetry);
                robot.Transfer.setPowerDirect(0.0);
            }
            // Phase B: Actuate transfer motor gate kicker forward to punch shot (351ms to 700ms)
            else {
                robot.FlyWheel.run(gamepad1, gamepad2, currentDistance, nectarIsNext, telemetry);
                robot.Transfer.setPowerDirect(1.0);
            }
        } else {
            // Sequence completed successfully, stop everything and shift states
            robot.FlyWheel.start(0);
            robot.Transfer.setPowerDirect(0);
            setNextState(nextState);
        }
    }
}
