package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.FCDrivingWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.RevIMUv2;

@TeleOp(name="Robot Linear OpMode", group="test")
public class DriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {

        FCDrivingWrapper drivingWrapper = new FCDrivingWrapper(hardwareMap, telemetry);
        RevIMUv2 imu = new RevIMUv2(hardwareMap, "imu");
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);

        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            drivingWrapper.drive(imu, joystickWrapper, 1, 1);

            robot.execute(telemetry);

            telemetry.addData("State", robot.getCurrentState().toString());
            telemetry.update();
        }
    }
}
