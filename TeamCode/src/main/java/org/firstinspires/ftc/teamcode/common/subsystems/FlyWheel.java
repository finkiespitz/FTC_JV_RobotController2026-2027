package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.Robot;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
public class FlyWheel {
    private double tuningVelocity = 1060;
    public static double pollenFactor = 2;
    private static double kp = 40;
    private static double kf = 15;
    private static int stepIndex = 0;
    private final double[] stepSizes = {0.01, 0.1, 1, 10};
    private final DcMotorEx flyWheel;
    private double flyWheelVelocity = 0;
   private boolean flyWheelOn = false;

    public FlyWheel(HardwareMap hardwareMap) {

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kp, 0, 0, kf);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flyWheel.setDirection(DcMotor.Direction.FORWARD);


    }

    //Use the Gamepads
    public void run(Gamepad gamepad1, Gamepad gamepad2,
                    double distanceFromHive, Robot robot, Telemetry telemetry) {

        if(gamepad2.bWasPressed() && stepIndex < 3){
            stepIndex = stepIndex + 1;}

        if(gamepad2.xWasPressed() && stepIndex > 0){
            stepIndex = stepIndex -1;}

        if (gamepad2.yWasPressed()) {
            kp += stepSizes[stepIndex];}

        if (gamepad2.aWasPressed()) {
            kp -= stepSizes[stepIndex];}

        if (gamepad2.dpadUpWasPressed()) {
            kf +=stepSizes[stepIndex];}

        if (gamepad2.dpadDownWasPressed()) {
            kf -= stepSizes[stepIndex];}

        if (gamepad2.dpadRightWasPressed()) {
            tuningVelocity +=stepSizes[stepIndex];}

        if (gamepad2.dpadLeftWasPressed()) {
            tuningVelocity -= stepSizes[stepIndex];}


        if (gamepad2.touchpadWasPressed()) {
            flyWheelOn = !flyWheelOn;}

        if(flyWheelOn){
         flyWheelVelocity = tuningVelocity;
        }
        else flyWheelVelocity = 0;


        //auto transfer start for testing
        /**if(isSpunUp() && flyWheelVelocity > 10 && !gamepad2.right_trigger_pressed && !gamepad2.left_trigger_pressed){
            robot.Transfer.setPowerDirect(1);
        } else if (!gamepad2.right_trigger_pressed && !gamepad2.left_trigger_pressed) {robot.Transfer.setPowerDirect(0);
        }**/






        //applying the power to the flywheel
        flyWheel.setVelocity(flyWheelVelocity);


        //adding controller rumble
        if (flyWheelVelocity > 0) {
            gamepad2.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            gamepad1.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else{
            gamepad2.stopRumble();
            gamepad1.stopRumble();
        }
        telemetry.addData("kp", kp);
        telemetry.addData("kf", kf);
        telemetry.addData("stepSizes", stepSizes[stepIndex]);
        telemetry.addData("FlyWheel Velocity", flyWheel.getVelocity());
        telemetry.addData("flyWheel Target Velocity", tuningVelocity);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kp, 0, 0, kf);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    // Bypasses gamepad
    public void runAuto(double velocity) {
        flyWheel.setVelocity(velocity);
    }
    public void stop() {
        flyWheel.setPower(0);
    }

    public Boolean isSpunUp(){
        return flyWheel.getVelocity() > flyWheelVelocity - 50 && flyWheelVelocity < flyWheel.getVelocity() + 300;
    }

}

