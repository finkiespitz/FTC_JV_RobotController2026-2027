package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.acmerobotics.dashboard.config.Config;

@Config
public class FlyWheel {
    private static double nectarFactor = 0.30;
    private static double pollenFactor = 0.10;
    private final DcMotor flyWheel;
    private boolean previousXButton2;

    public FlyWheel(HardwareMap hardwareMap) {

        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");


    }

    // 1. TELOP METHOD:Use the Gamepads Directly
    public void run(Gamepad gamepad1, Gamepad gamepad2,
                    double distanceFromHive, boolean nectarIsNext, Telemetry telemetry) {

        boolean flyWheelOn = false;
        double flyWheelPower;
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
        flyWheel.setPower(flyWheelPower);


        //adding controller rumble
        if (flyWheelPower > 0) {
            gamepad2.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            gamepad1.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        } else{
            gamepad2.stopRumble();
            gamepad1.stopRumble();
        }

        telemetry.addData("flyWheel Power", flyWheelPower);
    }

    // 2. AUTONOMOUS METHOD: Bypasses gamepad and targets a direct speed
    public void start(double power) {
        if (power > 1) power = 1;
        if (power < -1) power = -1;
        flyWheel.setPower(power);
    }
}

