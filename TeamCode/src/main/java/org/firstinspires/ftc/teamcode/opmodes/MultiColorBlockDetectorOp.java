package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.robot.MultiColorSampleDetector;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.states.AutoPickupState;
import org.firstinspires.ftc.teamcode.interfaces.IRobot;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.opencv.core.Rect;

@TeleOp(name = "Multi-Color Block Detector", group = "Vision")
public class MultiColorBlockDetectorOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        telemetry.addLine("Initializing robot hardware...");
        telemetry.update();

        JoystickWrapper joystickWrapper = new JoystickWrapper(gamepad1, gamepad2);

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry);

        MultiColorSampleDetector sampleDetector = new MultiColorSampleDetector(hardwareMap, telemetry);

        telemetry.addLine("Initialization complete");
        telemetry.addLine("Press START to begin");
        telemetry.update();

        waitForStart();

        boolean isInAutoMode = false;
        boolean wasAPressed = false;
        boolean wasBPressed = false;

        while (opModeIsActive()) {
            boolean isAPressed = gamepad1.a;
            boolean isBPressed = gamepad1.b;

            if (isAPressed && !wasAPressed) {
                isInAutoMode = !isInAutoMode;

                if (isInAutoMode) {
                    robot.switchState(IRobot.State.AUTO_PICKUP);
                    telemetry.addLine("Auto pickup mode ENABLED");
                } else {
                    robot.switchState(IRobot.State.INTAKINGCLAW);
                    telemetry.addLine("Auto pickup mode DISABLED");
                }
            }

            if (isBPressed && !wasBPressed && isInAutoMode) {
                isInAutoMode = false;
                robot.switchState(IRobot.State.INTAKINGCLAW);
                telemetry.addLine("Auto pickup CANCELED");
            }

            wasAPressed = isAPressed;
            wasBPressed = isBPressed;

            if (!isInAutoMode) {
                sampleDetector.getCounts();

                Rect closest = sampleDetector.detectClosestSample();
                if (closest != null) {
                    telemetry.addData("Closest Sample",
                            String.format("Color: %s, X: %d, Y: %d",
                                    sampleDetector.getLastDetectedColor(),
                                    closest.x + closest.width/2,
                                    closest.y + closest.height/2));
                }

                telemetry.addLine("Press A to enable auto pickup");
            } else {
                robot.execute(telemetry);
            }

            telemetry.update();
        }

        sampleDetector.stopStreaming();
    }
}