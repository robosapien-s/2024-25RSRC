package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;

@TeleOp
public class SoumilTest extends LinearOpMode {
    DcMotor testMotor;
    @Override
    public void runOpMode() throws InterruptedException {
        testMotor = hardwareMap.get(DcMotor.class, "testMotor");

        waitForStart();

        while(!isStopRequested()) {
            if (gamepad1.a) {
                testMotor.setPower(1);
                telemetry.addData("A-Pressed: ", true);
            } else {
                testMotor.setPower(0);
                telemetry.addData("A-Pressed: ", false);
            }
            telemetry.update();
        }


    }
}
