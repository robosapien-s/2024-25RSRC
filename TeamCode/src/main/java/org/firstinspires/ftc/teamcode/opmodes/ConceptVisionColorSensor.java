package org.firstinspires.ftc.teamcode.opmodes;

import android.app.Activity;
import android.graphics.Color;
import android.view.View;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "Concept: Vision Color-Sensor")
public class ConceptVisionColorSensor extends LinearOpMode
{
    public String sensorColor() {
        // Create a color sensor reference
        NormalizedColorSensor colorSensor = hardwareMap.get(NormalizedColorSensor.class, "sensor_color");

        // Variables for gain adjustment and HSV color values
        float gain = 2.0f;
        final float[] hsvValues = new float[3];

        // Enable the sensor's light if it supports SwitchableLight
        if (colorSensor instanceof SwitchableLight) {
            ((SwitchableLight) colorSensor).enableLight(true);
        }

        // Adjust sensor gain (if needed)
        colorSensor.setGain(gain);

        // Retrieve the normalized colors from the sensor
        NormalizedRGBA colors = colorSensor.getNormalizedColors();

        // Convert the colors to HSV
        Color.colorToHSV(colors.toColor(), hsvValues);

        // Analyze the hue to determine the color
        float hue = hsvValues[0];
        String detectedColor;

        if ((hue >= 0 && hue <= 30) || (hue >= 330 && hue <= 360)) {
            detectedColor = "Red"; // Hue range for red
        } else if (hue >= 210 && hue <= 270) {
            detectedColor = "Blue"; // Hue range for blue
        } else if (hue >= 45 && hue <= 75) {
            detectedColor = "Yellow"; // Hue range for yellow
        } else {
            detectedColor = "None"; // No specified color detected
        }

        // Return the detected color as a string
        return detectedColor;
    }

    @Override
    public void runOpMode() {
        String color;
        while(true){
            color = sensorColor();
            telemetry.addData("color",color);
            telemetry.update();
        }

    }
}
