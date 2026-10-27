package org.firstinspires.ftc.teamcode.common.subsystems;

// 1. PLACE IMPORTS AT THE VERY TOP OF THE FILE
import android.graphics.Color;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class NectarSensor {
    private final ColorSensor colorSensor;

    // Minimum light level (Value/Brightness) to ensure a game piece is actually in the tray
    private static final int COLOR_THRESHOLD = 300;

    public NectarSensor(HardwareMap hardwareMap) {
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
    }

    public boolean run(Telemetry telemetry) {
        // Read raw channel values
        int red = colorSensor.red();
        int blue = colorSensor.blue();
        int green = colorSensor.green();

        boolean nectarDetected = false;

        float[] hsv = new float[3];
        // Multiplying raw values by 8 scales them properly into the standard Android color space
        Color.RGBToHSV(red * 8, green * 8, blue * 8, hsv);

        float hue = hsv[0];        // Extracted Hue color wheel angle (0 to 360 degrees)
        float saturation = hsv[1]; // Extracted saturation (0.0 to 1.0)
        float value = hsv[2];      // Extracted total brightness/lux value

        // Verify a physical object is present by checking total light intensity reflection
        boolean pieceIsPresent = (red > COLOR_THRESHOLD || green > COLOR_THRESHOLD || blue > COLOR_THRESHOLD);

        if (pieceIsPresent) {
            // Check for Yellow Pollen
            if (hue >= 40 && hue <= 70) {
                nectarDetected = false;
            }
            // If a piece is present but not yellow, check for Alliance Nectar (Red or Blue)
            else if ((hue >= 0 && hue <= 25) || (hue >= 330 && hue <= 360)) {
                nectarDetected = true;
            }
            else if (hue >= 190 && hue <= 250) {
                nectarDetected = true;
            }
        }


        if (pieceIsPresent) {
            telemetry.addData("Nectar Detected", nectarDetected ? "TRUE" : "FALSE (POLLEN)");
        } else {
            telemetry.addData("Nectar Detected", "FALSE (EMPTY TRAY)");
        }

        // Extremely useful live dashboard tuning tool for tournament field calibrations
        telemetry.addData("Live Sensor Hue", "%.1f deg", hue);

        return nectarDetected;
    }
}