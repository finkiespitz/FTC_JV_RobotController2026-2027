package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.utils.Timer; // Plural version verified for your project configuration
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.common.Robot;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pedroPaths.PedroPathBlue1;
import org.firstinspires.ftc.teamcode.common.pedroPathing.pedroPaths.PedroPathRed1;

@Autonomous(name = "Auto Red1", group = "Autonomous")
public class AutoRed extends LinearOpMode {

    //private Variables v
    private Follower follower;
    private Robot robot;
    private PedroPathRed1 paths;
    private Timer stateTimer;

    // Set to true to filter the Red alliance Hive tags correctly during autonomous shooting
    private final boolean redTeam = true;
    private int shotsFiredInState = 0;

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
        paths = new PedroPathRed1();
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
            double elapsedSeconds = stateTimer.seconds();

            switch (currentState) {

                case SHOOT_PRELOADS:
                    // Auto-shoots the 4 preloaded pieces first
                    runAutoShooterSequence(4, (long) elapsedMillis, AutoState.DRIVE_TO_GARDEN);
                    break;

                case DRIVE_TO_GARDEN:
                    if (elapsedSeconds < 0.1) {
                        follower.follow(paths.gardenIntake());
                        robot.Intake.setPowerDirect(1.0);
                    }

                    // Safety guard: Ensure pathing has actually registered as busy before checking completion
                    if (elapsedSeconds > 0.3 && !follower.isBusy()) {
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

                    if (follower.completion() >= 0.80) {
                        double currentDistance = robot.Limelight.getPose(redTeam);
                        boolean nectarNext = robot.NectarSensor.run(telemetry);
                        robot.FlyWheel.runAuto(currentDistance, nectarNext);
                    }

                    // Arrived at the Hive shooting station
                    if (elapsedSeconds > 0.3 && !follower.isBusy()) {
                        robot.Intake.setPowerDirect(0.0);
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

                    if (elapsedSeconds > 0.3 && !follower.isBusy()) {
                        setNextState(AutoState.DONE);
                    }
                    break;

                case DONE:
                    // Explicitly isolate power channels to prevent motor stall heating
                    robot.FlyWheel.stop();
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
        shotsFiredInState = 0;
        stateTimer.reset();
    }

    /**
     * Automates individual game-piece firing cycles dynamically.
     */
    private void runAutoShooterSequence(int targetCount, long currentMillis, AutoState nextState) {
        double currentDistance = robot.Limelight.getPose(redTeam);
        boolean nectarNext = robot.NectarSensor.run(telemetry);

        long singleShotTime = 700; // 350ms spool-up + 350ms feed window duration
        long totalSequenceTimeNeeded = targetCount * singleShotTime;

        if (currentMillis < totalSequenceTimeNeeded) {
            // Determine which specific shot number we are on based on time elapsed
            shotsFiredInState = (int) (currentMillis / singleShotTime);
            long timeIntoCurrentShot = currentMillis % singleShotTime;

            // Phase A: Rev shooter wheels up using distance maps (0ms to 349ms)
            if (timeIntoCurrentShot < 350) {
                robot.FlyWheel.runAuto(currentDistance, nectarNext);
                robot.Transfer.setPowerDirect(0.0);
            }
            // Phase B: Actuate transfer motor gate kicker forward to punch shot (350ms to 699ms)
            else {
                robot.FlyWheel.runAuto(currentDistance, nectarNext);
                robot.Transfer.setPowerDirect(1.0);
            }
        } else {
            robot.FlyWheel.stop();
            robot.Transfer.setPowerDirect(0);
            setNextState(nextState);
        }
    }
}
