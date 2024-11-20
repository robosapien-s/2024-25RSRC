package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.FCDrive;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp
public class TestFCDrive extends LinearOpMode {

    FCDrive drive;
    JoystickWrapper joystickWrapper;
    @Override
    public void runOpMode() throws InterruptedException {
        joystickWrapper = new JoystickWrapper(gamepad1,gamepad2);
        drive = new FCDrive(hardwareMap, RevHubOrientationOnRobot.LogoFacingDirection.UP, RevHubOrientationOnRobot.UsbFacingDirection.RIGHT, true);
        waitForStart();
        while (!isStopRequested()) {
            drive.update(telemetry, joystickWrapper, 1, 1);
        }
    }
}
