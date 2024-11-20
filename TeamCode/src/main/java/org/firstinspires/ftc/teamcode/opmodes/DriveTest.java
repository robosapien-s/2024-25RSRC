package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.FCDrivingWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.RevIMUv2;

@Config

@TeleOp(name="Robot Linear OpMode", group="test")
public class DriveTest extends LinearOpMode {

    public static class Params {
        public static final double CLAW_OPEN = 0.3;
        public static final double CLAW_CLOSE = 0.67;

        public static final double ROT_SERVO_DEFAULT = 0.4;

        public static final double CLAW_ANGLE_FORWARD = 1.0;
        public static final double CLAW_ANGLE_DOWN = 0.5;
        public static final double CLAW_ANGLE_BACK = 0.0;

        public static final int CLAW_SLIDER_FORWARD = 0;
        public static final int CLAW_SLIDER_DOWN = 2000;
        public static final int CLAW_SLIDER_BACK = -21110;

        public static final int HORIZONTAL_SLIDE_INTAKE_INITIAL = 200;

        public static final double INTAKE_ANGLE_TRANSFER = .48;
        public static final double INTAKE_ANGLE_DOWN = .51;
        public static final int VERTICAL_SLIDE_POSITION = 140;

        public static final int VERTICAL_SLIDE_TRANSFER_POSITION = 420;
        public static final int HORIZONTAL_SLIDE_TRANSFER_POSITION = 100;
    }

    public static DriveTest.Params PARAMS = new DriveTest.Params();

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2);



        waitForStart();

        while (opModeIsActive()) {
            robot.execute(telemetry);

            telemetry.addData("State", robot.getCurrentState().toString());
           // telemetry.addData("Vertical slide", )
            telemetry.update();
        }
    }
}
