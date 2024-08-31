package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Robot Linear OpMode", group="Linear Opmode")
public class DriveTest extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = new Robot();

        waitForStart();

        while (opModeIsActive()) {
            robot.execute();

            telemetry.addData("State", robot.getState().toString());
            telemetry.update();
        }
    }
}
