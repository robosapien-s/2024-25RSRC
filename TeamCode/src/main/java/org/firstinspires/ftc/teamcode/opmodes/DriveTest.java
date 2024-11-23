package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Const;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.robot.Constants;
import org.firstinspires.ftc.teamcode.robot.Robot;
import org.firstinspires.ftc.teamcode.wrappers.FCDrivingWrapper;
import org.firstinspires.ftc.teamcode.wrappers.JoystickWrapper;
import org.firstinspires.ftc.teamcode.wrappers.RevIMUv2;

@Config

@TeleOp(name="Robot Linear OpMode", group="test")
public class DriveTest extends LinearOpMode {

    public static class Params {
        public static double CLAW_OPEN = Constants.CLAW_OPEN;
        public static double CLAW_CLOSE = Constants.CLAW_CLOSE;

        public static double ROT_SERVO_DEFAULT = Constants.ROT_SERVO_DEFAULT;
        public static double ROT_SERVO_BACK = Constants.ROT_SERVO_BACK;

        public static double CLAW_ANGLE_FORWARD = Constants.CLAW_ANGLE_FORWARD;
        public static double CLAW_ANGLE_DOWN = Constants.CLAW_ANGLE_DOWN;
        public static double CLAW_ANGLE_BACK = Constants.CLAW_ANGLE_BACK;

        public static int CLAW_SLIDER_FORWARD = Constants.CLAW_SLIDER_FORWARD;
        public static int CLAW_SLIDER_DOWN = Constants.CLAW_SLIDER_DOWN;
        public static int CLAW_SLIDER_BACK = Constants.CLAW_SLIDER_BACK;

        public static int HORIZONTAL_SLIDE_INTAKE_INITIAL = Constants.HORIZONTAL_SLIDE_INTAKE_INITIAL;

        public static double INTAKE_ANGLE_TRANSFER = Constants.INTAKE_ANGLE_TRANSFER;
        public static double INTAKE_ANGLE_DOWN = Constants.INTAKE_ANGLE_DOWN;
        public static int VERTICAL_SLIDE_POSITION = Constants.VERTICAL_SLIDE_POSITION;

        public static int VERTICAL_SLIDE_TRANSFER_POSITION = Constants.VERTICAL_SLIDE_TRANSFER_POSITION;

        public static int VERTICAL_SLIDE_WALL_POSITION = Constants.VERTICAL_SLIDE_WALL_POSITION;

        public static int VERTICAL_SLIDE_HANG_PREP_POSITION = Constants.VERTICAL_SLIDE_HANG_PREP_POSITION;

        public static int VERTICAL_SLIDE_HANG_DROP_POSITION = Constants.VERTICAL_SLIDE_HANG_DROP_POSITION;

        public static int VERTICAL_SLIDE_DROP_L1 = Constants.VERTICAL_SLIDE_DROP_L1;

        public static int VERTICAL_SLIDE_DROP_L2 = Constants.VERTICAL_SLIDE_DROP_L2;

        public static int HORIZONTAL_SLIDE_TRANSFER_POSITION = Constants.HORIZONTAL_SLIDE_TRANSFER_POSITION;

        public static int HORIZONTAL_SLIDE_MAX_POSITION = Constants.HORIZONTAL_SLIDE_MAX_POSITION;

        public static double VERTICAL_SLIDE_PID_kp = Constants.VERTICAL_SLIDE_PID_kp;
        public static double VERTICAL_SLIDE_PID_kd = Constants.VERTICAL_SLIDE_PID_kd;
        public static double VERTICAL_SLIDE_PID_ki = Constants.VERTICAL_SLIDE_PID_ki;

        public static int VERTICAL_SLIDE_PID_target = Constants.VERTICAL_SLIDE_PID_target;

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
