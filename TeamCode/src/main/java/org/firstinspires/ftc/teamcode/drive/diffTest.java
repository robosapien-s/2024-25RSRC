package org.firstinspires.ftc.teamcode.drive;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.wrappers.DiffArmWrapper;

@TeleOp
public class diffTest extends LinearOpMode {
    GamepadEx gamepad1;
    GamepadEx gamepad2;
    @Override
    public void runOpMode() throws InterruptedException {
        DiffArmWrapper diffArmWrapper = new DiffArmWrapper(hardwareMap,telemetry);
        waitForStart();
        while (!isStopRequested()) {
            gamepad1.readButtons();
            gamepad2.readButtons();
            if (gamepad1.wasJustPressed(GamepadKeys.Button.A)) {
                diffArmWrapper.changeCurrMode();
            }
            diffArmWrapper.move(gamepad1.getLeftY(), gamepad1.getRightY());
        }
    }
}
