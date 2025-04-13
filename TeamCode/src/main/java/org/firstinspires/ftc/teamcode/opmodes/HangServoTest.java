package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp
public class HangServoTest extends LinearOpMode {

    Servo hangServo;
    double increment = 0.05;

    double targetPos = 0.5;

    JoystickWrapper joystick;


    @Override
    public void runOpMode() throws InterruptedException {
        hangServo = hardwareMap.get(Servo.class, "hangServo");
        joystick = new JoystickWrapper(gamepad1, gamepad2);
        waitForStart();

        hangServo.setPosition(targetPos);

        while (!isStopRequested()) {
            if (joystick.gamepad1GetDUp()) {
                targetPos+=increment;
                hangServo.setPosition(targetPos);
            } else if (joystick.gamepad1GetDDown()) {
                targetPos-=increment;
                hangServo.setPosition(targetPos);
            } else if (joystick.gamepad1GetDRight()) {
                increment*=2;
            } else if (joystick.gamepad1GetDLeft()) {
                increment/=2;
            }
            telemetry.addData("increment", increment);
            telemetry.addData("target pos", targetPos);
            telemetry.addData("servo pos", hangServo.getPosition());
            telemetry.update();

            //.3 up
            //.825 down
        }
    }
}
