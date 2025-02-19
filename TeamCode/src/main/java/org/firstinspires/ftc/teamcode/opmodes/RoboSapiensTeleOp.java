package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.teamcode.Localizer;
import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.ThreeDeadWheelLocalizer;
import org.firstinspires.ftc.teamcode.TwoDeadWheelLocalizer;
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
        public static double CLAW_ANGLE_BUCKET = 0.23;
        public static double CLAW_ANGLE_BACK = 0.14;

        public static double CLAW_ANGLE_PREP_BACK = 0.5;
        public static double CLAW_ANGLE_TRANSFER = .76;

        public static int CLAW_SLIDER_FORWARD = 12000;
        public static int CLAW_SLIDER_TRANSFER = 0;
        public static int CLAW_SLIDER_DOWN = 6000;
        public static int CLAW_SLIDER_BACK = 0;

        public static int HORIZONTAL_SLIDE_INTAKE_INITIAL = 200;

        public static double INTAKE_ANGLE_DOWN = .51;
        public static int VERTICAL_SLIDE_POSITION = 140;

        public static int VERTICAL_SLIDE_TRANSFER_POSITION = 0;

        public static int VERTICAL_SLIDE_MIDDLE_POSITION = 500;

        public static int VERTICAL_SLIDE_WALL_POSITION = 200;

        public static int VERTICAL_SLIDE_DOWN_POSITION = 0;

        public static int VERTICAL_SLIDE_HANG_PREP_POSITION = 1030;

        public static int VERTICAL_SLIDE_HANG_DROP_POSITION = 600;

        public static int VERTICAL_SLIDE_DROP_L1 = 2450;

        public static int VERTICAL_SLIDE_DROP_L2 = 3000;

        public static int VERTICAL_SLIDE_MAX_POSITION = 3000;

        public static int HORIZONTAL_SLIDE_TRANSFER_POSITION = 0;

        public static int HORIZONTAL_SLIDE_MAX_POSITION = 1650;

        public static double VERTICAL_SLIDE_PID_kp = 0.027;
        public static double VERTICAL_SLIDE_PID_kd = 0.027;
        public static double VERTICAL_SLIDE_PID_ki = 0.0001;

        public static int VERTICAL_SLIDE_PID_target = 400;

        public static double INTAKE_CLAW_OPEN = .49;
        public static double INTAKE_CLAW_CLOSE = .55;
        public static double INTAKE_CLAW_LOOSE = .53;

        public static double INTAKE_ROT_SERVO_DEFAULT = .2393;

        public static double INTAKE_ANGLE_READY = .3;//.6183;
        //public static double INTAKE_ANGLE_READY_LOW = .3394;//.6183;
        public static double INTAKE_ANGLE_PICKUP =  .43;//.68;

        public static double INTAKE_ANGLE_TRANSFER = .14;

        public static double INTAKE_KNUCKLE_PICKUP = 0.1789;//.1;
        public static double INTAKE_KNUCKLE_TRANSFER = 1;

        public static double CLAW_HORIZONTAL_ANGLE_CENTER = 0.54;
        public static double CLAW_HORIZONTAL_ANGLE_LEFT = 0.42;
        public static double CLAW_HORIZONTAL_ANGLE_RIGHT = 0.66;



        public static double ANGLE_DRIVE_KP = 0.026;
        public static double ANGLE_DRIVE_KD = .00004;
        public static double ANGLE_DRIVE_KI = 0.0;

        public static double ANGLE_DRIVE_TARGET_TEST = 0.0;

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
//            telemetry.addData("x pos", localizer.getPose().position.x);
//            telemetry.addData("y pos", localizer.getPose().position.y);
//            telemetry.addData("angle (deadwheels)", Math.toDegrees(localizer.getPose().heading.toDouble()));
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
