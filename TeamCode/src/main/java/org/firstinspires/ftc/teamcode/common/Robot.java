package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.common.subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.common.subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.common.subsystems.Limelight;
import org.firstinspires.ftc.teamcode.common.subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.common.subsystems.NectarSensor;
import org.firstinspires.ftc.teamcode.common.subsystems.Transfer;
import org.firstinspires.ftc.teamcode.common.tuning.Localization;

import com.qualcomm.robotcore.hardware.HardwareMap;



public class Robot {
    // Subsystem objects
    public DriveTrain driveTrain;
    public Intake Intake;
    public Limelight Limelight;
    public FlyWheel FlyWheel;
    public NectarSensor NectarSensor;
    public FlowerIntake FlowerIntake;
    public Localization Localization;
    public Transfer Transfer;

    // Constructor initializes all your subsystems at once
    public Robot(HardwareMap hardwareMap, Telemetry telemetry) {
        this.Localization = new Localization(hardwareMap, telemetry);
        this.driveTrain = new DriveTrain(Localization, hardwareMap);
        this.Intake = new Intake(hardwareMap);
        this.Limelight = new Limelight(hardwareMap);
        this.FlyWheel = new FlyWheel(hardwareMap);
        this.NectarSensor = new NectarSensor(hardwareMap);
        //this.FlowerIntake = new FlowerIntake(hardwareMap);
        this.Transfer = new Transfer (hardwareMap);
    }
}