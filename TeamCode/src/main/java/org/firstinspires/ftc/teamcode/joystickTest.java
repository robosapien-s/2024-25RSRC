package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.arcrobotics.ftclib.gamepad.KeyReader;
import com.arcrobotics.ftclib.gamepad.ToggleButtonReader;
import com.arcrobotics.ftclib.gamepad.TriggerReader;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class joystickTest extends LinearOpMode {
    GamepadEx gamepad1;
    GamepadEx gamepad2;
    @Override
    public void runOpMode() throws InterruptedException {

        TriggerReader rightTrigger = new TriggerReader(gamepad1, GamepadKeys.Trigger.RIGHT_TRIGGER);

        ToggleButtonReader bToggle = new ToggleButtonReader(gamepad1, GamepadKeys.Button.B);

        waitForStart();

        while (!isStopRequested()) {
            gamepad1.readButtons(); //TODO: Must be called every loop for it to function, also for gp2

            if (gamepad1.wasJustPressed(GamepadKeys.Button.A)) { //basically our joystick wrapper previousl
                telemetry.addData("Falling Edge", true);
            }

            if (gamepad1.wasJustReleased(GamepadKeys.Button.A)) {
                telemetry.addData("Rising Edge", true);
            }

            if (gamepad1.stateJustChanged(GamepadKeys.Button.A)) { //TODO: Falling edge or rising edge
                telemetry.addData("State Changed", true);
            }

            telemetry.addData("leftStickY", gamepad1.getLeftY());
            /**
             * IMPORTANT: Check if ^^^^ is flipped -- traditionally up is negative for the y sticks, but
             * it seems like they reversed it for convenience
             */

            telemetry.addData("rightTrigger", gamepad1.getTrigger(GamepadKeys.Trigger.RIGHT_TRIGGER));
            //TODO: Returns actual value of trigger

            if (rightTrigger.isDown()) { //TODO: trigger is pressed if the value is greater than .5
                telemetry.addData("rightTrigger pressed?", true);
            }

            //TODO: Coolest one IMO --> Button Toggle (declared above for g1B)

            telemetry.addData("bToggle", bToggle.getState());

            telemetry.update();

        }

    }
}
