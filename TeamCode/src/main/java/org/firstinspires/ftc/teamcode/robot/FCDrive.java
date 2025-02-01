package org.firstinspires.ftc.teamcode.robot;

import android.graphics.Point;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.geometry.euclidean.threed.Vector3D;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.wrappers.FCDrivingWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.RevIMUv2;
import org.firstinspires.ftc.teamcode.wrappers.RevIMUv3;

import kotlin.NotImplementedError;

public class FCDrive implements IDrive {
        private FCDrivingWrapper drivingWrapper;
        private RevIMUv3 imu;

        public FCDrive(HardwareMap hardwareMap) {
                //Default Control Hub orientation (main bot)
                drivingWrapper = new FCDrivingWrapper(hardwareMap);
                imu = new RevIMUv3(hardwareMap, "imu");
                imu.init();
        }

        public FCDrive(HardwareMap hardwareMap, RevHubOrientationOnRobot.LogoFacingDirection logoDirection, RevHubOrientationOnRobot.UsbFacingDirection usbDirection, boolean demoBot) {
                //Specificed Control Hub orientation (secondary bot)
                //For main swap defaults in RevIMUv3
                drivingWrapper = new FCDrivingWrapper(hardwareMap, demoBot);
                imu = new RevIMUv3(hardwareMap, "imu");
                imu.init(logoDirection, usbDirection);
        }

        @Override
        public void setTargetHeading(double heading) {

        }

        @Override
        public Vector3D getRobotPosition() {
                return null;
        }

        @Override
        public void setAutoMode(double inX, double inY) {
        }

        public void update(Telemetry telemetry, JoystickWrapper joystickWrapper, double speed, double rotSpeed) {
                drivingWrapper.drive(telemetry, imu, joystickWrapper, speed, rotSpeed);
        }

        @Override
        public void updateRaw(Telemetry telemetry, boolean isLeftStickPressed, double leftStickX, double leftStickY, double rightStickX, double rightStickY, double speed, double rotSpeed) {
                throw new UnsupportedOperationException("Not implemented");
        }
}
