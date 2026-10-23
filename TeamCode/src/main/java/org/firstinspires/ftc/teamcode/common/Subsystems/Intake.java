package org.firstinspires.ftc.teamcode.common.Subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class Intake {
    private DcMotor intakeMotor;

    public Intake(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
    }

    // 1. TELEOP CALLS THIS: Utilizes Gamepads
    public void run(Gamepad gamepad2,
                    Telemetry telemetry) {
        double intakePower = 0;

        if (gamepad2.right_trigger_pressed) {
            intakePower = gamepad2.right_trigger;
        } else if (gamepad2.left_trigger_pressed) {
            intakePower = gamepad2.left_trigger;
        }

        intakeMotor.setPower(intakePower);

        }
    // 2. AUTONOMOUS CALLS THIS: Bypasses gamepads entirely
    // Positive power to intake, negative power to reverse/outtake, 0 to stop
    public void setPowerDirect(double power) {
        intakeMotor.setPower(power);


    }
}

