package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp
public class ServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        CRServo servo = hardwareMap.get(CRServo.class, "servo");
        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1,gamepad2);
        waitForStart();

        while (!isStopRequested()) {
            servo.setPower(joystickWrapper.gamepad1GetRightStickY());
            telemetry.addData("Servo Power", joystickWrapper.gamepad1GetRightStickY());
            telemetry.update();
        }
    }
}
