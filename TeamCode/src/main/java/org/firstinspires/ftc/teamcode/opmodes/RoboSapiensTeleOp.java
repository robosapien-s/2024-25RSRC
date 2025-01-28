package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.robot.Robot;

@Config

@TeleOp(name="Robot Linear OpMode", group="test")
public class RoboSapiensTeleOp extends LinearOpMode {

    public static class Params {
        public static double CLAW_OPEN = 0.76;
        public static double CLAW_CLOSE = 1;

        public static double ROT_SERVO_DEFAULT = .74;
        public static double ROT_SERVO_BACK = .08;

        public static double CLAW_ANGLE_FORWARD = .66;

        public static double CLAW_ANGLE_FORWARD_SPECIMEN = .48;
        public static double CLAW_ANGLE_DOWN = 0.96;
        public static double CLAW_ANGLE_BACK = 0.14;

        public static double CLAW_ANGLE_PREP_BACK = 0.5;
        public static double CLAW_ANGLE_TRANSFER = .8;

        public static int CLAW_SLIDER_FORWARD = 12000;
        public static int CLAW_SLIDER_TRANSFER = 2650;
        public static int CLAW_SLIDER_DOWN = 6000;
        public static int CLAW_SLIDER_BACK = 0;

        public static int HORIZONTAL_SLIDE_INTAKE_INITIAL = 200;

        public static double INTAKE_ANGLE_DOWN = .51;
        public static int VERTICAL_SLIDE_POSITION = 140;

        public static int VERTICAL_SLIDE_TRANSFER_POSITION = 400;

        public static int VERTICAL_SLIDE_WALL_POSITION = 200;

        public static int VERTICAL_SLIDE_DOWN_POSITION = 0;

        public static int VERTICAL_SLIDE_HANG_PREP_POSITION = 1362;

        public static int VERTICAL_SLIDE_HANG_DROP_POSITION = 800;

        public static int VERTICAL_SLIDE_DROP_L1 = 2350;

        public static int VERTICAL_SLIDE_DROP_L2 = 4100;

        public static int VERTICAL_SLIDE_MAX_POSITION = 2350;

        public static int HORIZONTAL_SLIDE_TRANSFER_POSITION = 0;

        public static int HORIZONTAL_SLIDE_MAX_POSITION = 1650;

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
        public static double INTAKE_ANGLE_PICKUP =  .43;//.68;

        public static double INTAKE_ANGLE_TRANSFER = .255;

        public static double INTAKE_KNUCKLE_PICKUP = 0.1789;//.1;
        public static double INTAKE_KNUCKLE_TRANSFER = .97;

        public static double CLAW_HORIZONTAL_ANGLE_CENTER = 0.54;
        public static double CLAW_HORIZONTAL_ANGLE_LEFT = 0.42;
        public static double CLAW_HORIZONTAL_ANGLE_RIGHT = 0.66;


    }

    public static RoboSapiensTeleOp.Params PARAMS = new RoboSapiensTeleOp.Params();

    @Override
    public void runOpMode() {

        Robot robot = new Robot(hardwareMap, gamepad1, gamepad2, telemetry);

        //0
        DcMotorEx frontLeftMotor = hardwareMap.get(DcMotorEx.class,"fL");
        //1
        DcMotorEx frontRightMotor = hardwareMap.get(DcMotorEx.class,"fR");
        //2
        DcMotorEx backLeftMotor = hardwareMap.get(DcMotorEx.class,"bL");
        //3
        DcMotorEx backRightMotor = hardwareMap.get(DcMotorEx.class,"bR");


        DcMotorEx horizontalSlide = hardwareMap.get(DcMotorEx.class, "horizontalSlide1");
        DcMotorEx verticalSlide1 = hardwareMap.get(DcMotorEx.class, "verticalSlide1");
        DcMotorEx verticalSlide2 = hardwareMap.get(DcMotorEx.class, "verticalSlide2");


        waitForStart();

        while (opModeIsActive()) {
            robot.execute(telemetry);

//            telemetry.addData("FrontLeftMotor current (A): ", frontLeftMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("FrontRightMotor current (A): ", frontRightMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("BackLeftMotor current (A): ", backLeftMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("BackRightMotor current (A): ", backRightMotor.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("horizontalSlide current (A): ", horizontalSlide.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("verticalSlide1 current (A): ", verticalSlide1.getCurrent(CurrentUnit.AMPS));
//            telemetry.addData("verticalSlide2 current (A): ", verticalSlide2.getCurrent(CurrentUnit.AMPS));
//
//            telemetry.update();

           // telemetry.addData("State", robot.getCurrentState().toString());
           // telemetry.addData("Vertical slide", )
           // telemetry.update();
        }
    }
}
