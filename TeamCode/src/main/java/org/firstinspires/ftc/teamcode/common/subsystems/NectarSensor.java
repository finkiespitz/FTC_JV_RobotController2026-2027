package org.firstinspires.ftc.teamcode.common.subsystems;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class NectarSensor {
    private final ColorSensor colorSensor;

    // Threshold value to determine if a color is strong enough to count
    // (Adjust this number up or down depending on your field lighting)
    private static final int COLOR_THRESHOLD = 300;

    public NectarSensor(HardwareMap hardwareMap) {
        // Change "colorSensor" to match your Driver Station configuration name
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public boolean run(Telemetry telemetry) {
        // Read raw channel values from the color sensor
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();

        boolean nectarDetected = false;

        // 1. CHECK FOR POLLEN FIRST:
        boolean isPollen = (red > COLOR_THRESHOLD && green > COLOR_THRESHOLD && blue < red - 150);

        if (isPollen) {
            nectarDetected = false; // It's pollen! Return false so the flywheel stays at pollen speed.
        }

        // 2. CHECK FOR NECTAR (RED OR BLUE):
        // If Red is significantly higher than Blue, it's Red Nectar.
        // If Blue is significantly higher than Red, it's Blue Nectar.
        if (red > blue + 150 && red > COLOR_THRESHOLD) {
            nectarDetected = true; // Red Nectar detected!
        }

        if (blue > red + 150 && blue > COLOR_THRESHOLD) {
            nectarDetected = true; // Blue Nectar detected!
        }

        telemetry.addData("nectar", nectarDetected);
        // Default fallback if the tray is completely empty
        return nectarDetected;
    }
}