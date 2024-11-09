package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp(name="Robot Linear OpMode", group="test")
public class DriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);

        waitForStart();

        while (opModeIsActive()) {
            robot.execute(telemetry);

            telemetry.addData("State", robot.getCurrentState().toString());
            telemetry.update();
        }
    }
}
