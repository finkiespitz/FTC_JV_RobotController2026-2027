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
        // If Y is held down, run the motor at full power. Otherwise, stop it completely.
        if (gamepad2.a) {
            transferMotor.setPower(1.0);
        } else {
            transferMotor.setPower(0.0);
        }

        telemetry.addData("Transfer Motor Running", gamepad2.a);
    }

    public void setPowerDirect(double power) {
        transferMotor.setPower(power);
    }
}