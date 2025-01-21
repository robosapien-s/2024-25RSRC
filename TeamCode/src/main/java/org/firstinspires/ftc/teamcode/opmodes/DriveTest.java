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
        public static double CLAW_OPEN = 0.36;
        public static double CLAW_CLOSE = 0.6;

        public static double ROT_SERVO_DEFAULT = .74;
        public static double ROT_SERVO_BACK = .08;

        public static double CLAW_ANGLE_FORWARD = .66;

        public static double CLAW_ANGLE_FORWARD_SPECIMEN = .48;
        public static double CLAW_ANGLE_DOWN = 0.96;
        public static double CLAW_ANGLE_BACK = 0.14;

        public static double CLAW_ANGLE_TRANSFER = .48;

        public static int CLAW_SLIDER_FORWARD = 12000;
        public static int CLAW_SLIDER_TRANSFER = 2000;
        public static int CLAW_SLIDER_DOWN = 6000;
        public static int CLAW_SLIDER_BACK = 0;

        public static int HORIZONTAL_SLIDE_INTAKE_INITIAL = 200;

        public static double INTAKE_ANGLE_DOWN = .51;
        public static int VERTICAL_SLIDE_POSITION = 140;

        public static int VERTICAL_SLIDE_TRANSFER_POSITION = 420;

        public static int VERTICAL_SLIDE_WALL_POSITION = 275;

        public static int VERTICAL_SLIDE_DOWN_POSITION = 120;

        public static int VERTICAL_SLIDE_HANG_PREP_POSITION = 1450;

        public static int VERTICAL_SLIDE_HANG_DROP_POSITION = 1175;

        public static int VERTICAL_SLIDE_DROP_L1 = 1800;

        public static int VERTICAL_SLIDE_DROP_L2 = 4100;

        public static int VERTICAL_SLIDE_MAX_POSITION = 2100;

        public static int HORIZONTAL_SLIDE_TRANSFER_POSITION = 210;

        public static int HORIZONTAL_SLIDE_MAX_POSITION = 400;

        public static double VERTICAL_SLIDE_PID_kp = 0.027;
        public static double VERTICAL_SLIDE_PID_kd = 0.027;
        public static double VERTICAL_SLIDE_PID_ki = 0.0001;

        public static int VERTICAL_SLIDE_PID_target = 400;



        public static double INTAKE_CLAW_OPEN = .48;
        public static double INTAKE_CLAW_CLOSE = .53;
        public static double INTAKE_CLAW_LOOSE = .5175;


        public static double INTAKE_ROT_SERVO_DEFAULT = .2393;


        public static double INTAKE_ANGLE_READY = .3;//.6183;
        //public static double INTAKE_ANGLE_READY_LOW = .3394;//.6183;
        public static double INTAKE_ANGLE_PICKUP =  .5;//.68;

        public static double INTAKE_ANGLE_TRANSFER = .35;

        public static double INTAKE_KNUCKLE_PICKUP = 0.1789;//.1;
        public static double INTAKE_KNUCKLE_TRANSFER = .78;


    }

    public static DriveTest.Params PARAMS = new DriveTest.Params();

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry);



        waitForStart();

        while (opModeIsActive()) {
            robot.execute(telemetry);

            telemetry.addData("State", robot.getCurrentState().toString());
           // telemetry.addData("Vertical slide", )
            telemetry.update();
        }
    }
}
