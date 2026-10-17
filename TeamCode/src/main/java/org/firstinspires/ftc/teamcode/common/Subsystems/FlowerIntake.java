package org.firstinspires.ftc.teamcode.common.Subsystems;

import androidx.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;


public class FlowerIntake {
    private DcMotor linearSlide;
    private boolean prevDPadUp;
    private boolean linearSlideFullyExtended;


    public FlowerIntake( HardwareMap hardwareMap) {
        linearSlide = hardwareMap.get(DcMotor.class, "flowerIntakeSlide");
        linearSlide.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    // 1. TELEOP CALLS THIS: Uses gamepad triggers
    public void run( Gamepad gamepad2,
                     Telemetry telemetry) {


        if(gamepad2.dpad_up && !prevDPadUp){
            linearSlide.setTargetPosition(linearSlideFullyExtended ? 0 : 1000);
        }
        prevDPadUp = gamepad2.dpad_up;
        linearSlideFullyExtended = linearSlide.getTargetPosition() > 1;

        linearSlide.setPower(1);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        telemetry.addData("LinearSlide Pos", linearSlide.getCurrentPosition());
        telemetry.update();

    }
    // 2. AUTONOMOUS CALLS THIS: Bypasses gamepads entirely
    // True = extend to 1000, False = retract to 0
    public void setExtendedDirect(boolean extend, double power) {
        int target = extend ? 1000 : 0;
        linearSlide.setTargetPosition(target);
        linearSlide.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        linearSlide.setPower(power);
    }

    // Helper for Auto to check if the slide is still traveling
    public boolean isBusy() {
        return linearSlide.isBusy();
    }
}