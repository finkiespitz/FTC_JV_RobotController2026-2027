package org.firstinspires.ftc.teamcode.common.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.teamcode.common.Robot;

public class FlyWheel {
    private DcMotor flyWheel;
    private boolean previousXButton2;

    public FlyWheel(HardwareMap hardwareMap) {

        flyWheel = hardwareMap.get(DcMotor.class, "flyWheel");


    }

    public void run(HardwareMap hardwareMap, double nectarFactor, double pollenFactor, Gamepad gamepad1, Gamepad gamepad2) {

        Robot robot = new Robot(hardwareMap);

        double distanceFromHive = robot.Limelight.run();

        boolean nectarIsNext = robot.NectarSensor.run();


        double flyWheelPower = 0;


        double nectarSpeed = distanceFromHive * nectarFactor;
        double pollenSpeed = distanceFromHive * pollenFactor;

        if(gamepad2.x && !previousXButton2) {
             flyWheelPower = nectarIsNext ? nectarSpeed : pollenSpeed;
        }
        else  flyWheelPower = 0;

        previousXButton2 = gamepad2.x;

        //locking flyWheelPower to 1
        if(flyWheelPower > 1)flyWheelPower = 1;

        //applying the power to the flywheel
        flyWheel.setPower(flyWheelPower);


        //adding controller rumble
        if(flyWheelPower > 0) {
            gamepad2.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
            gamepad1.rumble(1.0, 1.0, Gamepad.RUMBLE_DURATION_CONTINUOUS);
        }
        else gamepad2.stopRumble(); gamepad1.stopRumble();



    }
}
