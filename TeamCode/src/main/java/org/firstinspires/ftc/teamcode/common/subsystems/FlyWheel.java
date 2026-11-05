package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

import java.util.function.BooleanSupplier;

@Config
public class FlyWheel {
    public static double nectarFactor = 3;
    public static double pollenFactor = 2;
    private static double kp = 0.0;
    private static double kf = 0.0;
    private static int stepIndex = 0;
    private final double[] stepSizes = {0.01, 0.1, 1, 10};
    private final DcMotorEx flyWheel;
    private boolean previousXButton2;
    private double flyWheelPower = 0;
   private boolean flyWheelOn = false;

    public FlyWheel(HardwareMap hardwareMap) {

        flyWheel = hardwareMap.get(DcMotorEx.class, "flyWheel");

        PIDFCoefficients pidfCoefficients = new PIDFCoefficients(kp, 0, 0, kf);
        flyWheel.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, pidfCoefficients);
        flyWheel.setDirection(DcMotor.Direction.REVERSE);


    }

    // 1. TELOP METHOD:Use the Gamepads Directly
    public void run(Gamepad gamepad1, Gamepad gamepad2,
                    double distanceFromHive, boolean nectarIsNext, Telemetry telemetry) {

        if(gamepad2.xWasPressed() && stepIndex < 4){
            stepIndex = stepIndex + 1;}

        if(gamepad2.bWasPressed() && stepIndex > 0){
            stepIndex = stepIndex -1;}

        if (gamepad2.yWasPressed()) {
            kp += stepSizes[stepIndex];}

        if (gamepad2.aWasPressed()) {
            kp -= stepSizes[stepIndex];}

        if (gamepad2.dpadUpWasPressed()) {
            kf +=stepSizes[stepIndex];}

        if (gamepad2.dpadDownWasPressed()) {
            kf -= stepSizes[stepIndex];}


        double nectarSpeed = distanceFromHive * nectarFactor;
        double pollenSpeed = distanceFromHive * pollenFactor;

        if (gamepad2.x && !previousXButton2) {
            flyWheelOn = !flyWheelOn;}

        if(flyWheelOn){
         flyWheelPower = nectarIsNext ? nectarSpeed : pollenSpeed;
        }
        else flyWheelPower = 0;

        previousXButton2 = gamepad2.x;

        //locking flyWheelPower to 1
        if (flyWheelPower > 1) flyWheelPower = 1;

        //applying the power to the flywheel
        flyWheel.setVelocity(flyWheelPower);


        //adding controller rumble
        if (flyWheelPower > 0) {
            gamepad2.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            gamepad1.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else{
            gamepad2.stopRumble();
            gamepad1.stopRumble();
        }
        telemetry.addData("kp", kp);
        telemetry.addData("kf", kf);
        telemetry.addData("stepSizes", stepSizes[stepIndex]);
        telemetry.addData("FlyWheel Velocity", "%.2f", flyWheel.getVelocity());
        telemetry.addData("FlyWheel Target Velocity", "%.2f", flyWheel.getTargetPosition());
        telemetry.addData("flyWheel Velocity", "%.2f", flyWheelPower);
    }

    // 2. AUTONOMOUS METHOD: Bypasses gamepad
    public void runAuto(double distanceFromHive, boolean nectarNext) {
        double nectarSpeed = distanceFromHive * nectarFactor;
        double pollenSpeed = distanceFromHive * pollenFactor;

        double flyWheelPower = nectarNext ? nectarSpeed : pollenSpeed;

        flyWheel.setPower(flyWheelPower);

    }
    public void stop() {
        flyWheel.setPower(0);
    }

    public Boolean isSpunUp(){
        return flyWheel.getVelocity() > flyWheel.getVelocity() - 0.1 && flyWheel.getVelocity() < flyWheel.getVelocity() + 0.2;
    }

}

