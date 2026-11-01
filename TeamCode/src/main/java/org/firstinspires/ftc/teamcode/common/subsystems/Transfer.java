package org.firstinspires.ftc.teamcode.common.subsystems;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Transfer {
    private final DcMotor transferMotor;

    public Transfer(HardwareMap hardwareMap) {

        transferMotor = hardwareMap.get(DcMotor.class, "transferMotor");

        transferMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public void run(Gamepad gamepad2, Telemetry telemetry) {
        double transferPower = 0;

        if (gamepad2.right_trigger_pressed) {
            transferPower = 1;
        }

        if (gamepad2.left_trigger_pressed) {
            transferPower = -1;
        }
        transferMotor.setPower(transferPower);

        telemetry.addData("Transfer Motor Running", gamepad2.a);
    }

    public void setPowerDirect(double power) {
        transferMotor.setPower(power);
    }
}