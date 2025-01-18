package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.robot.DualServoSlideController;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp(name="DualServoTest2", group="demo")
public class DualServoTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {

        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);
        DualServoSlideController controller = new DualServoSlideController(hardwareMap, "servo1", "servo2", "motor1", 12000, 0 );
        waitForStart();

        while (!isStopRequested()) {

            if(joystickWrapper.gamepad1GetB()) {
                controller.setTargetPosition(11600);
            } else if(joystickWrapper.gamepad1GetX()) {
                controller.setTargetPosition(0);
            }

            controller.update(telemetry);


        }
    }
}
