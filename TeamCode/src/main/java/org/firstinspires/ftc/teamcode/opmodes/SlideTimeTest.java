package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.Robot;

@TeleOp
public class SlideTimeTest extends LinearOpMode {

    Robot robot;
    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry);

        long startTime;
        long endTime = 0;

        boolean reached = false;
        waitForStart();


        startTime = System.currentTimeMillis();
        robot.setHorizontalSlideTargetPosition(1650);
        while (!isStopRequested()) {
            if (!reached && robot.getHorizontalSlidePosition()>1645) {
                reached = true;
                endTime = System.currentTimeMillis();
            }

            telemetry.addData("reached", reached);
            if (reached) {
                telemetry.addData("time taken", endTime-startTime);
            }

            robot.execute(telemetry);

        }

    }
}
