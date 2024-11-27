package org.firstinspires.ftc.teamcode.usefulStuff;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

public class AngleThing {


    HardwareMap hardwareMap;
    Telemetry telemetry;
    IMU imu;

    double pitch;
    double roll;
    double yaw;
    double range;

    double PGain = .03;

    double targetHeading;

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.hardwareMap = hardwareMap;
        this.telemetry = telemetry;
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);

        // Now initialize the IMU with this mounting orientation
        // This sample expects the IMU to be in a REV Hub and named "imu".
        imu = hardwareMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
    }
    public void angleStuff(JoystickWrapper joystickWrapper){
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();

        pitch = orientation.getPitch(AngleUnit.DEGREES);
        roll = orientation.getRoll(AngleUnit.DEGREES);
        yaw = orientation.getYaw(AngleUnit.DEGREES);

        if (length(joystickWrapper.gamepad1GetRightStickX(), -joystickWrapper.gamepad1GetRightStickY()) > .5) {
            targetHeading = Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetRightStickY(), joystickWrapper.gamepad1GetRightStickX()));
        }  // Save for telemetry

        // Determine the heading current error
        double headingError = targetHeading - yaw;

        // Normalize the error to be within +/- 180 degrees
        while (headingError > 180) headingError -= 360;
        while (headingError <= -180) headingError += 360;
        range = Range.clip(headingError * PGain, -1, 1);

        telemetry.addData("Joystick Left Angle", targetHeading);
        telemetry.addData("Joystick Right Angle", Math.toDegrees(Math.atan2(-joystickWrapper.gamepad1GetRightStickY(), -joystickWrapper.gamepad1GetRightStickX())));
        telemetry.addData("Pitch", pitch);
        telemetry.addData("Roll", roll);
        telemetry.addData("Yaw", yaw);
        telemetry.addData("Error", headingError);
        telemetry.addData("Range",range);
        telemetry.update();



    }
    public double getRange(){
        return range;
    }

    double length(double x, double y) {
        return Math.sqrt(x * x + y * y);
    }

}