package org.firstinspires.ftc.teamcode.common;

import org.firstinspires.ftc.teamcode.common.Subsystems.FlowerIntake;
import org.firstinspires.ftc.teamcode.common.Subsystems.Intake;
import org.firstinspires.ftc.teamcode.common.Subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.common.Subsystems.Limelight;
import org.firstinspires.ftc.teamcode.common.Subsystems.FlyWheel;
import org.firstinspires.ftc.teamcode.common.Subsystems.NectarSensor;

import com.qualcomm.robotcore.hardware.HardwareMap;



public class Robot {
    // Subsystem objects
    public DriveTrain driveTrain;
    public Intake Intake;
    public Limelight Limelight;
    public FlyWheel FlyWheel;
    public NectarSensor NectarSensor;
    public FlowerIntake FlowerIntake;

    // Constructor initializes all your subsystems at once
    public Robot(HardwareMap hardwareMap) {
        this.driveTrain = new DriveTrain(hardwareMap);
        this.Intake = new Intake(hardwareMap);
        this.Limelight = new Limelight(hardwareMap);
        this.FlyWheel = new FlyWheel(hardwareMap);
        this.NectarSensor = new NectarSensor(hardwareMap);
        this.FlowerIntake = new FlowerIntake(hardwareMap);

    }
}