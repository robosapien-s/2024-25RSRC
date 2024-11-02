package org.firstinspires.ftc.teamcode.opmodes;

import static android.os.SystemClock.sleep;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDevice;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
//import com.qualcomm.robotcore.hardware.I2cDeviceSynchImpl;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
@TeleOp
public class UltrasonicTest extends LinearOpMode {

    private I2cDeviceSynch ultrasonicSensor;

    private static final I2cAddr SENSOR_ADDRESS = I2cAddr.create7bit(0x70);
    private I2cDeviceSynch device;
    private I2cAddr i2cAddr;

    @Override
    public void runOpMode() throws InterruptedException {
        device = hardwareMap.get(I2cDeviceSynch.class, "ultrasonic");
        //ultrasonicSensor = new I2cDeviceSynchImpl(device, SENSOR_ADDRESS, false);
        i2cAddr = I2cAddr.create7bit(0x70); // Default address of MB1242

        device.engage();

        waitForStart();

        while(!isStopRequested()) {
            byte[] distance = device.read(0x00, 2);  // Reads two bytes starting from register 0x00
            int distanceInCm = (distance[0] << 8) | (distance[1] & 0xFF);  // Combine bytes to get distance

            telemetry.addData("Distance (cm)", distanceInCm);
            telemetry.update();
        }
    }
}
