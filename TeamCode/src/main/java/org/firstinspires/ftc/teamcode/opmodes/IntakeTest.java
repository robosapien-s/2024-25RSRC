package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp(name="IntakeTest", group="demo")
public class IntakeTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Declare our motors
        // Make sure your ID's match your configuration
        CRServo intake1 = hardwareMap.crservo.get("intake1");
        CRServo intake2 = hardwareMap.crservo.get("intake2");

        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);


        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {

            if(joystickWrapper.gamepad1GetA()) {
                intake1.setPower(-1);
                intake2.setPower(-1);
            }

            if(joystickWrapper.gamepad1GetX()) {
                intake1.setPower(1);
                intake2.setPower(1);
            }

            if(joystickWrapper.gamepad1GetB()) {
                intake1.setPower(0);
                intake2.setPower(0);
            }
        }
    }


}
