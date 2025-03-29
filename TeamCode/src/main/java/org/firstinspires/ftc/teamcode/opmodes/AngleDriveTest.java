package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.robot.AngleDrive;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;

@TeleOp
public class AngleDriveTest extends LinearOpMode {

    AngleDrive drive;
    JoystickWrapper joystickWrapper;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new AngleDrive(hardwareMap, false);
        joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        waitForStart();

        while (!isStopRequested()) {
            drive.update(telemetry, joystickWrapper, 1,1);
        }

    }
}
