package org.firstinspires.ftc.teamcode.common.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FlowerIntake {
    private Servo servo;
    private boolean prevDPadUp;


    public FlowerIntake( HardwareMap hardwareMap) {
        servo = hardwareMap.get(Servo.class, "flowerServo");
        servo.setPosition(0);
    }
    public void run( Gamepad gamepad2,
                     Telemetry telemetry) {
        if(gamepad2.dpad_up && !prevDPadUp) {
            if (servo.getPosition() == 0) {
                servo.setPosition(10);
            } else {
                servo.setPosition(0);}}



        telemetry.addData("flowerServo Pose", servo.getPosition());

        telemetry.update();
    }
}