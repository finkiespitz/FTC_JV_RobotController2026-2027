package org.firstinspires.ftc.teamcode.common.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    public void run(Gamepad gamepad2) {
        double intakePower = 0;

        if (gamepad2.right_trigger_pressed) {
            intakePower = gamepad2.right_trigger;
        } else if (gamepad2.left_trigger_pressed) {
            intakePower = gamepad2.left_trigger;
        }

        intakeMotor.setPower(intakePower);
    }
}
