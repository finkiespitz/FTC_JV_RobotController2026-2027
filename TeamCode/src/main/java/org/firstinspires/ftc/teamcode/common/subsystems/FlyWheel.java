package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;


@Config
public class FlyWheel {
    public static double pollenFactor = 2;
    private static double kp = 0.0;
    private static double kf = 0.0;
    private static int stepIndex = 0;
    private final double[] stepSizes = {0.01, 0.1, 1, 10};
    private final DcMotorEx flyWheel;
    private double flyWheelVelocity = 0;
   private boolean flyWheelOn = false;

    public FlyWheel(HardwareMap hardwareMap) {

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kp, 0, 0, kf);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);


    }

    //Use the Gamepads
    public void run(Gamepad gamepad1, Gamepad gamepad2,
                    double distanceFromHive, Telemetry telemetry) {

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


        if (gamepad2.touchpadWasPressed()) {
            flyWheelOn = !flyWheelOn;}

        if(flyWheelOn){
         flyWheelVelocity = distanceFromHive * pollenFactor;;
        }
        else flyWheelVelocity = 0;



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
        telemetry.addData("flyWheel Target Velocity", flyWheelVelocity);

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kp, 0, 0, kf);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
    }

    // Bypasses gamepad
    public void runAuto(double distanceFromHive) {
        double flyWheelVelocity = distanceFromHive * pollenFactor;

        flyWheel.setPower(flyWheelVelocity);

    }
    public void stop() {
        flyWheel.setPower(0);
    }

    public Boolean isSpunUp(){
        return flyWheel.getVelocity() > flyWheelVelocity - 10 && flyWheelVelocity < flyWheel.getVelocity() + 10;
    }

}

