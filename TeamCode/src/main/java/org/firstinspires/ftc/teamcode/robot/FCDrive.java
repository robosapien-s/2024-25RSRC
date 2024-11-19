package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.interfaces.IDrive;
import org.firstinspires.ftc.teamcode.wrappers.FCDrivingWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.RevIMUv2;
import org.firstinspires.ftc.teamcode.wrappers.RevIMUv3;

public class FCDrive implements IDrive {
        private FCDrivingWrapper drivingWrapper;
        private RevIMUv3 imu;


        public FCDrive(HardwareMap hardwareMap) {
                drivingWrapper = new FCDrivingWrapper(hardwareMap);
                imu = new RevIMUv3(hardwareMap, "imu");
        }

        public void init() {
                imu.init();
        }

        public void update(Telemetry telemetry, JoystickWrapper joystickWrapper, double speed, double rotSpeed) {
                drivingWrapper.drive(telemetry, imu, joystickWrapper, speed, rotSpeed);
        }
}
